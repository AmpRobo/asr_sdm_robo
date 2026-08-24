#include "asr_sdm_log_collector/log_collector_node.hpp"

#include "asr_sdm_log_collector/log_record.hpp"
#include "asr_sdm_log_collector/path_utils.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace asr_sdm::log
{

namespace
{

spdlog::level::level_enum parseLevel(const std::string & text, spdlog::level::level_enum fallback)
{
  const auto level = spdlog::level::from_str(text);
  if (level == spdlog::level::off && text != "off") {
    return fallback;
  }
  return level;
}

spdlog::level::level_enum levelFromRosout(uint8_t rosout_level)
{
  using rcl_interfaces::msg::Log;
  if (rosout_level >= Log::FATAL) {
    return spdlog::level::critical;
  }
  if (rosout_level >= Log::ERROR) {
    return spdlog::level::err;
  }
  if (rosout_level >= Log::WARN) {
    return spdlog::level::warn;
  }
  if (rosout_level >= Log::INFO) {
    return spdlog::level::info;
  }
  if (rosout_level >= Log::DEBUG) {
    return spdlog::level::debug;
  }
  return spdlog::level::trace;
}

/// Socket modes are naturally written in octal, but YAML's handling of a leading
/// zero is not portable enough to rely on, so the parameter is a string.
unsigned int parsePermissions(const std::string & text, unsigned int fallback)
{
  try {
    return static_cast<unsigned int>(std::stoul(text, nullptr, 8));
  } catch (const std::exception &) {
    return fallback;
  }
}

}  // namespace

LogCollectorNode::LogCollectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("asr_sdm_log_collector", options)
{
  LogWriter::Config writer_config;
  writer_config.log_directory = declare_parameter<std::string>("sink.log_directory", "");
  writer_config.use_run_subdirectory = declare_parameter<bool>("sink.use_run_subdirectory", true);
  writer_config.log_filename = declare_parameter<std::string>("sink.log_filename", "all.log");
  writer_config.per_node_files = declare_parameter<bool>("sink.per_node_files", true);
  writer_config.echo_to_console = declare_parameter<bool>("sink.echo_to_console", false);
  writer_config.max_file_size_bytes =
    static_cast<std::size_t>(declare_parameter<int>("sink.max_file_size_mb", 100)) * 1024UL *
    1024UL;
  writer_config.max_files = static_cast<std::size_t>(declare_parameter<int>("sink.max_files", 10));
  writer_config.max_tracked_nodes =
    static_cast<std::size_t>(declare_parameter<int>("sink.max_tracked_nodes", 256));
  writer_config.flush_level =
    parseLevel(declare_parameter<std::string>("sink.flush_level", "warning"), spdlog::level::warn);
  writer_config.file_pattern =
    declare_parameter<std::string>("sink.file_pattern", writer_config.file_pattern);
  writer_config.console_pattern =
    declare_parameter<std::string>("sink.console_pattern", writer_config.console_pattern);

  unknown_node_name_ = declare_parameter<std::string>("unknown_node_name", "unknown");
  minimum_level_ =
    parseLevel(declare_parameter<std::string>("minimum_level", "trace"), spdlog::level::trace);

  writer_ = std::make_unique<LogWriter>(writer_config);
  if (!writer_->directoryWarning().empty()) {
    RCLCPP_WARN(get_logger(), "%s", writer_->directoryWarning().c_str());
  }
  RCLCPP_INFO(get_logger(), "Collecting logs into %s", writer_->directory().c_str());

  DatagramReceiver::Options receiver_options;
  receiver_options.max_datagram_bytes =
    static_cast<std::size_t>(declare_parameter<int>("max_datagram_bytes", 65536));
  receiver_options.receive_buffer_bytes =
    declare_parameter<int>("receive_buffer_bytes", 4 * 1024 * 1024);

  const bool unix_enabled = declare_parameter<bool>("unix_socket.enabled", true);
  const auto unix_socket_path =
    declare_parameter<std::string>("unix_socket.socket_path", "~/log/vehicle/log.sock");
  const auto unix_permissions = declare_parameter<std::string>("unix_socket.permissions", "0666");

  if (unix_enabled) {
    DatagramReceiver::UnixEndpoint endpoint;
    endpoint.socket_path = expandUserPath(unix_socket_path);
    endpoint.permissions = parsePermissions(unix_permissions, 0666);
    startReceiver(endpoint, receiver_options, "unix_socket.socket_path");
  }

  const bool udp_enabled = declare_parameter<bool>("udp.enabled", true);
  const auto udp_bind_address = declare_parameter<std::string>("udp.bind_address", "0.0.0.0");
  const auto udp_port = declare_parameter<int>("udp.port", 9110);

  if (udp_enabled) {
    DatagramReceiver::UdpEndpoint endpoint;
    endpoint.bind_address = udp_bind_address;
    endpoint.port = static_cast<uint16_t>(udp_port);
    startReceiver(endpoint, receiver_options, "udp.port");
  }

  const bool rosout_enabled = declare_parameter<bool>("rosout.enabled", true);
  const auto rosout_topic = declare_parameter<std::string>("rosout.topic", "/rosout");
  const auto rosout_queue_depth = declare_parameter<int>("rosout.queue_depth", 1000);
  ignore_own_rosout_ = declare_parameter<bool>("rosout.ignore_own_logs", true);
  own_logger_name_ = get_logger().get_name();

  if (rosout_enabled) {
    // /rosout is published reliably with transient-local durability; a volatile
    // reliable subscription is compatible and skips the startup backlog.
    rclcpp::QoS rosout_qos{rclcpp::KeepLast(static_cast<std::size_t>(rosout_queue_depth))};
    rosout_qos.reliable();

    rosout_subscription_ = create_subscription<rcl_interfaces::msg::Log>(
      rosout_topic, rosout_qos,
      [this](const rcl_interfaces::msg::Log::SharedPtr message) { onRosout(message); });
    RCLCPP_INFO(get_logger(), "Collecting ROS logs from %s", rosout_topic.c_str());
  }

  const auto flush_period = declare_parameter<double>("sink.flush_period_sec", 2.0);
  if (flush_period > 0.0) {
    flush_timer_ = create_wall_timer(
      std::chrono::duration<double>{flush_period}, [this]() { writer_->flush(); });
  }

  const auto statistics_period = declare_parameter<double>("statistics_period_sec", 30.0);
  if (statistics_period > 0.0) {
    statistics_timer_ = create_wall_timer(
      std::chrono::duration<double>{statistics_period}, [this]() { reportStatistics(); });
  }
}

LogCollectorNode::~LogCollectorNode()
{
  // Stop feeding the writer before it goes away.
  for (auto & receiver : receivers_) {
    receiver->stop();
  }
  rosout_subscription_.reset();
  if (writer_) {
    writer_->flush();
  }
}

void LogCollectorNode::onDatagram(const char * data, std::size_t size)
{
  // Runs on the receiver thread. Nothing here may log through RCLCPP_*, or the
  // collector would feed its own /rosout intake.
  const LogRecord record = parseDatagram(data, size, unknown_node_name_);
  if (record.level < minimum_level_) {
    filtered_records_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  writer_->write(record);
  datagram_records_.fetch_add(1, std::memory_order_relaxed);
}

void LogCollectorNode::onRosout(const rcl_interfaces::msg::Log::SharedPtr message)
{
  if (ignore_own_rosout_ && message->name == own_logger_name_) {
    return;
  }

  LogRecord record;
  record.level = levelFromRosout(message->level);
  if (record.level < minimum_level_) {
    filtered_records_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  // A zero stamp means the clock was not running yet, for instance under sim
  // time before the first /clock message.
  if (message->stamp.sec == 0 && message->stamp.nanosec == 0) {
    record.time = std::chrono::system_clock::now();
  } else {
    record.time = std::chrono::system_clock::time_point{
      std::chrono::seconds{message->stamp.sec} + std::chrono::nanoseconds{message->stamp.nanosec}};
  }

  record.logger_name = message->name.empty() ? unknown_node_name_ : message->name;
  record.file = message->file;
  record.line = static_cast<int>(message->line);
  record.function = message->function;
  record.message = message->msg;

  writer_->write(record);
  rosout_records_.fetch_add(1, std::memory_order_relaxed);
}

void LogCollectorNode::reportStatistics()
{
  const auto counts = writer_->perNodeCounts();

  std::ostringstream sources;
  sources << "datagrams=" << datagram_records_.load(std::memory_order_relaxed)
          << " rosout=" << rosout_records_.load(std::memory_order_relaxed)
          << " filtered=" << filtered_records_.load(std::memory_order_relaxed);

  uint64_t socket_errors = 0;
  for (const auto & receiver : receivers_) {
    socket_errors += receiver->errorCount();
  }
  sources << " socket_errors=" << socket_errors;
  if (const auto unrouted = writer_->unroutedNodeCount(); unrouted > 0) {
    sources << " unrouted=" << unrouted;
  }

  RCLCPP_INFO(get_logger(), "Collected from %zu nodes (%s)", counts.size(), sources.str().c_str());
}

}  // namespace asr_sdm::log
