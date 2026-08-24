#ifndef ASR_SDM_LOG_COLLECTOR__LOG_CLIENT_HPP_
#define ASR_SDM_LOG_COLLECTOR__LOG_CLIENT_HPP_

#include <spdlog/spdlog.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace asr_sdm::log
{

struct ClientConfig
{
  /// Reported to the collector and used to name its per-node file. Use the ROS
  /// node name so the collected output lines up with the ROS graph.
  std::string node_name;

  /// Preferred transport when the collector is on this host. Used when the path
  /// is set and the socket already exists; otherwise the UDP target below is
  /// used, so a process started before the collector still logs somewhere.
  /// Must agree with the collector's `unix_socket.socket_path`, or every sender
  /// quietly falls back to UDP.
  std::string collector_socket_path{"~/log/vehicle/log.sock"};

  std::string collector_host{"127.0.0.1"};
  uint16_t collector_port{9110};

  spdlog::level::level_enum level{spdlog::level::info};
  spdlog::level::level_enum flush_level{spdlog::level::warn};

  bool log_to_console{true};
  bool log_to_collector{true};

  /// Records are handed to a background thread so a control loop never waits on
  /// a socket or a terminal. When the queue is full the oldest record is dropped
  /// instead of blocking the caller, which is the right trade for real-time code.
  std::size_t queue_size{8192};
  std::size_t worker_threads{1};

  std::size_t max_datagram_bytes{60000};

  std::string console_pattern{"[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%n] %v"};
};

/// Applies environment overrides on top of the defaults, so log verbosity and
/// the collector endpoint can be changed without touching launch files:
/// ASR_SDM_LOG_COLLECTOR_SOCKET, ASR_SDM_LOG_COLLECTOR_HOST,
/// ASR_SDM_LOG_COLLECTOR_PORT, ASR_SDM_LOG_LEVEL, ASR_SDM_LOG_FLUSH_LEVEL,
/// ASR_SDM_LOG_CONSOLE, ASR_SDM_LOG_TO_COLLECTOR, ASR_SDM_LOG_QUEUE_SIZE.
///
/// The socket path accepts `~` and `$VAR` references.
ClientConfig configFromEnv(std::string node_name);

/// Installs an async logger fanning out to the console and the collector, and
/// makes it spdlog's default so the SPDLOG_* macros use it.
///
/// Never throws: if the collector socket cannot be opened the logger is still
/// returned with console output only, because logging must not take a node down.
/// Calling it again returns the logger built by the first call.
std::shared_ptr<spdlog::logger> initialize(const ClientConfig & config);

/// Shorthand for initialize(configFromEnv(node_name)).
std::shared_ptr<spdlog::logger> initialize(const std::string & node_name);

/// Flushes and releases the logger. Call before leaving main so queued records
/// are not lost on exit.
void shutdown();

/// Records lost to a full queue or a refused socket, for health reporting.
uint64_t droppedRecordCount();

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__LOG_CLIENT_HPP_
