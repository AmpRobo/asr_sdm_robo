#ifndef ASR_SDM_LOG_COLLECTOR__LOG_WRITER_HPP_
#define ASR_SDM_LOG_COLLECTOR__LOG_WRITER_HPP_

#include "asr_sdm_log_collector/log_record.hpp"

#include <spdlog/formatter.h>
#include <spdlog/sinks/sink.h>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace asr_sdm::log
{

/// Persists collected records to rotating files: one file per originating node,
/// plus a merged file that interleaves every node in arrival order.
///
/// Records keep the timestamp, level, thread and source location of the process
/// that produced them, so the collected files read as if each node had written
/// them itself.
class LogWriter
{
public:
  struct Config
  {
    /// Accepts `~` and `$VAR` references. Empty, or not writable, derives the
    /// location from ROS_LOG_DIR, then ROS_HOME, then $HOME/.ros/log, matching
    /// where the rest of the ROS tooling writes.
    std::string log_directory;
    /// Group each run under its own timestamped subdirectory and point a
    /// `latest` symlink at it.
    bool use_run_subdirectory{true};
    /// Name of the file that interleaves every node.
    std::string log_filename{"all.log"};
    bool per_node_files{true};
    bool echo_to_console{false};
    std::size_t max_file_size_bytes{100UL * 1024UL * 1024UL};
    std::size_t max_files{10};
    /// Caps open file descriptors if a misbehaving sender invents node names.
    /// Records beyond the cap still reach the merged file.
    std::size_t max_tracked_nodes{256};
    spdlog::level::level_enum flush_level{spdlog::level::warn};
    /// `%*` expands to the originating process id and `%_` to the source
    /// location, neither of which stock spdlog patterns can express here.
    std::string file_pattern{"[%Y-%m-%d %H:%M:%S.%e] [%-8l] [%n] [%*:%t] %v%_"};
    std::string console_pattern{"[%H:%M:%S.%e] [%^%-8l%$] [%n] %v%_"};
  };

  explicit LogWriter(Config config);
  ~LogWriter();

  LogWriter(const LogWriter &) = delete;
  LogWriter & operator=(const LogWriter &) = delete;

  void write(const LogRecord & record);
  void flush();

  const std::filesystem::path & directory() const;

  /// Non-empty when `log_directory` could not be used and a fallback was taken,
  /// so the caller can say so loudly instead of writing somewhere unexpected in
  /// silence. Worth logging at warning level.
  const std::string & directoryWarning() const;

  /// Records written per originating node, for periodic health reporting.
  std::map<std::string, uint64_t> perNodeCounts() const;
  /// Nodes that exceeded `max_tracked_nodes` or whose file could not be opened.
  uint64_t unroutedNodeCount() const;

private:
  spdlog::sink_ptr sinkForNode(const std::string & node_name);
  std::unique_ptr<spdlog::formatter> makeFormatter(const std::string & pattern) const;
  void flushLocked();

  Config config_;
  std::filesystem::path directory_;
  std::string directory_warning_;
  mutable std::mutex mutex_;
  spdlog::sink_ptr merged_sink_;
  spdlog::sink_ptr console_sink_;
  std::map<std::string, spdlog::sink_ptr> node_sinks_;
  std::map<std::string, uint64_t> per_node_counts_;
  uint64_t unrouted_nodes_{0};
};

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__LOG_WRITER_HPP_
