#include "asr_sdm_log_collector/log_client.hpp"

#include "asr_sdm_log_collector/collector_sink.hpp"
#include "asr_sdm_log_collector/log_record.hpp"
#include "asr_sdm_log_collector/path_utils.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace asr_sdm::log
{

namespace
{

std::mutex g_mutex;
std::shared_ptr<spdlog::logger> g_logger;
std::shared_ptr<spdlog::details::thread_pool> g_thread_pool;
std::shared_ptr<CollectorSink> g_collector_sink;

const char * environmentValue(const char * name)
{
  const char * value = std::getenv(name);
  return (value != nullptr && *value != '\0') ? value : nullptr;
}

bool parseBoolean(const std::string & text, bool fallback)
{
  if (text == "1" || text == "true" || text == "True" || text == "on" || text == "yes") {
    return true;
  }
  if (text == "0" || text == "false" || text == "False" || text == "off" || text == "no") {
    return false;
  }
  return fallback;
}

spdlog::level::level_enum parseLevelOr(const std::string & text, spdlog::level::level_enum fallback)
{
  const auto level = spdlog::level::from_str(text);
  if (level == spdlog::level::off && text != "off") {
    return fallback;
  }
  return level;
}

/// Reports sink failures at most once a minute. A logging path that printed on
/// every failure would itself become the fault when the collector goes away.
void reportSinkError(const std::string & message)
{
  static std::atomic<int64_t> last_report_seconds{0};
  const auto now = std::chrono::duration_cast<std::chrono::seconds>(
                     std::chrono::steady_clock::now().time_since_epoch())
                     .count();
  int64_t previous = last_report_seconds.load(std::memory_order_relaxed);
  if (now - previous < 60) {
    return;
  }
  if (!last_report_seconds.compare_exchange_strong(previous, now, std::memory_order_relaxed)) {
    return;
  }
  std::cerr << "[asr_sdm_log_collector] log sink error: " << message << std::endl;
}

/// Opens the Unix socket when the collector has already created it, and falls
/// back to UDP otherwise. Returns nullptr when neither could be opened.
std::shared_ptr<CollectorSink> makeCollectorSink(const ClientConfig & config)
{
  CollectorSink::Options options;
  options.max_datagram_bytes = config.max_datagram_bytes;

  const std::string socket_path = expandUserPath(config.collector_socket_path);
  if (!socket_path.empty()) {
    std::error_code error;
    if (std::filesystem::exists(socket_path, error)) {
      try {
        return std::make_shared<CollectorSink>(CollectorSink::UnixTarget{socket_path}, options);
      } catch (const std::exception & failure) {
        std::cerr << "[asr_sdm_log_collector] " << socket_path
                  << " unusable, falling back to UDP: " << failure.what() << std::endl;
      }
    }
  }

  try {
    return std::make_shared<CollectorSink>(
      CollectorSink::UdpTarget{config.collector_host, config.collector_port}, options);
  } catch (const std::exception & failure) {
    std::cerr << "[asr_sdm_log_collector] collector sink disabled: " << failure.what() << std::endl;
    return nullptr;
  }
}

}  // namespace

ClientConfig configFromEnv(std::string node_name)
{
  ClientConfig config;
  config.node_name = std::move(node_name);

  if (const char * value = environmentValue("ASR_SDM_LOG_COLLECTOR_SOCKET")) {
    config.collector_socket_path = value;
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_COLLECTOR_HOST")) {
    config.collector_host = value;
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_COLLECTOR_PORT")) {
    try {
      config.collector_port = static_cast<uint16_t>(std::stoi(value));
    } catch (const std::exception &) {
      // Keep the default rather than refusing to log at all.
    }
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_LEVEL")) {
    config.level = parseLevelOr(value, config.level);
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_FLUSH_LEVEL")) {
    config.flush_level = parseLevelOr(value, config.flush_level);
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_CONSOLE")) {
    config.log_to_console = parseBoolean(value, config.log_to_console);
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_TO_COLLECTOR")) {
    config.log_to_collector = parseBoolean(value, config.log_to_collector);
  }
  if (const char * value = environmentValue("ASR_SDM_LOG_QUEUE_SIZE")) {
    try {
      config.queue_size = static_cast<std::size_t>(std::stoul(value));
    } catch (const std::exception &) {
      // Keep the default.
    }
  }

  return config;
}

std::shared_ptr<spdlog::logger> initialize(const ClientConfig & config)
{
  const std::lock_guard<std::mutex> guard{g_mutex};
  if (g_logger) {
    return g_logger;
  }

  const std::string logger_name = config.node_name.empty() ? "unknown" : config.node_name;

  std::vector<spdlog::sink_ptr> sinks;

  if (config.log_to_console) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_pattern(config.console_pattern);
    sinks.push_back(std::move(console_sink));
  }

  if (config.log_to_collector) {
    g_collector_sink = makeCollectorSink(config);
    if (g_collector_sink) {
      g_collector_sink->set_pattern(wire::pattern());
      sinks.push_back(g_collector_sink);
    }
  }

  const std::size_t queue_size = config.queue_size == 0 ? 8192 : config.queue_size;
  const std::size_t worker_threads = config.worker_threads == 0 ? 1 : config.worker_threads;
  g_thread_pool = std::make_shared<spdlog::details::thread_pool>(queue_size, worker_threads);

  g_logger = std::make_shared<spdlog::async_logger>(
    logger_name, sinks.begin(), sinks.end(), g_thread_pool,
    spdlog::async_overflow_policy::overrun_oldest);
  g_logger->set_level(config.level);
  g_logger->flush_on(config.flush_level);
  g_logger->set_error_handler(reportSinkError);

  spdlog::drop(logger_name);
  spdlog::register_logger(g_logger);
  spdlog::set_default_logger(g_logger);

  return g_logger;
}

std::shared_ptr<spdlog::logger> initialize(const std::string & node_name)
{
  return initialize(configFromEnv(node_name));
}

void shutdown()
{
  const std::lock_guard<std::mutex> guard{g_mutex};
  if (!g_logger) {
    return;
  }
  // Drains the queue while every sink is still alive.
  g_logger->flush();
  g_logger.reset();
  g_collector_sink.reset();
  // Releases the registry's references, the default logger among them.
  spdlog::shutdown();
  // Last owner, so the destructor joins the backend thread.
  g_thread_pool.reset();
}

uint64_t droppedRecordCount()
{
  const std::lock_guard<std::mutex> guard{g_mutex};
  uint64_t dropped = 0;
  if (g_thread_pool) {
    dropped += static_cast<uint64_t>(g_thread_pool->overrun_counter());
  }
  if (g_collector_sink) {
    dropped += g_collector_sink->droppedCount();
  }
  return dropped;
}

}  // namespace asr_sdm::log
