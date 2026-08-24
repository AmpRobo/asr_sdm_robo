#ifndef ASR_SDM_LOG_COLLECTOR__LOG_COLLECTOR_NODE_HPP_
#define ASR_SDM_LOG_COLLECTOR__LOG_COLLECTOR_NODE_HPP_

#include "asr_sdm_log_collector/datagram_receiver.hpp"
#include "asr_sdm_log_collector/log_writer.hpp"

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace asr_sdm::log
{

/// Aggregates the log output of every process on the robot into one place.
///
/// Three sources feed the same writer:
///   * A Unix socket, the preferred path for processes on this host using this
///     package's spdlog client. Works for non-ROS processes too.
///   * A UDP port, for senders on another machine.
///   * `/rosout`, so nodes that still log through RCLCPP_* are covered without
///     any change to their code.
class LogCollectorNode : public rclcpp::Node
{
public:
  explicit LogCollectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LogCollectorNode() override;

private:
  void onDatagram(const char * data, std::size_t size);
  void onRosout(const rcl_interfaces::msg::Log::SharedPtr message);
  void reportStatistics();

  /// Opens one transport and keeps going if it fails, so a socket path that is
  /// not writable on a development machine cannot stop the collector from doing
  /// the rest of its job. `parameter_name` is quoted in the error message so an
  /// operator knows which value to change.
  template <typename Endpoint>
  void startReceiver(
    const Endpoint & endpoint, const DatagramReceiver::Options & options,
    const char * parameter_name)
  {
    try {
      auto receiver = std::make_unique<DatagramReceiver>(
        endpoint, options, [this](const char * data, std::size_t size) { onDatagram(data, size); });
      receiver->start();
      RCLCPP_INFO(get_logger(), "Collecting datagrams on %s", receiver->description().c_str());
      receivers_.push_back(std::move(receiver));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Not collecting on this transport: %s. Change %s or disable it.",
        error.what(), parameter_name);
    }
  }

  std::unique_ptr<LogWriter> writer_;
  /// One entry per enabled transport; both feed onDatagram.
  std::vector<std::unique_ptr<DatagramReceiver>> receivers_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_subscription_;
  rclcpp::TimerBase::SharedPtr statistics_timer_;
  // The writer only flushes on its own at warn and above, so low-traffic info
  // records would otherwise sit in the file buffer until the process exits.
  rclcpp::TimerBase::SharedPtr flush_timer_;

  std::string unknown_node_name_;
  std::string own_logger_name_;
  bool ignore_own_rosout_{true};
  spdlog::level::level_enum minimum_level_{spdlog::level::trace};

  std::atomic<uint64_t> datagram_records_{0};
  std::atomic<uint64_t> rosout_records_{0};
  std::atomic<uint64_t> filtered_records_{0};
};

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__LOG_COLLECTOR_NODE_HPP_
