#include "asr_sdm_log_collector/log_writer.hpp"

#include "asr_sdm_log_collector/path_utils.hpp"

#include <spdlog/details/log_msg.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace asr_sdm::log
{

namespace
{

/// spdlog's log_msg has no field for the originating process id, so it travels
/// beside the record. Formatting runs on the thread that called sink->log(),
/// which is the same thread that set this, so a thread_local is enough.
thread_local int64_t t_origin_pid = 0;

class OriginPidFlag : public spdlog::custom_flag_formatter
{
public:
  void format(
    const spdlog::details::log_msg &, const std::tm &, spdlog::memory_buf_t & dest) override
  {
    const std::string text = std::to_string(t_origin_pid);
    dest.append(text.data(), text.data() + text.size());
  }

  std::unique_ptr<spdlog::custom_flag_formatter> clone() const override
  {
    return std::make_unique<OriginPidFlag>();
  }
};

/// Renders the source location together with its separator so that records
/// without one do not leave empty brackets behind.
class SourceLocationFlag : public spdlog::custom_flag_formatter
{
public:
  void format(
    const spdlog::details::log_msg & msg, const std::tm &, spdlog::memory_buf_t & dest) override
  {
    if (msg.source.empty()) {
      return;
    }
    const std::string text =
      " (" + std::string{msg.source.filename} + ":" + std::to_string(msg.source.line) + ")";
    dest.append(text.data(), text.data() + text.size());
  }

  std::unique_ptr<spdlog::custom_flag_formatter> clone() const override
  {
    return std::make_unique<SourceLocationFlag>();
  }
};

const char * environmentValue(const char * name)
{
  const char * value = std::getenv(name);
  return (value != nullptr && *value != '\0') ? value : nullptr;
}

/// Where the rest of the ROS tooling writes, used when `log_directory` is empty
/// or turns out to be unusable.
std::filesystem::path derivedBaseDirectory()
{
  if (const char * value = environmentValue("ROS_LOG_DIR")) {
    return std::filesystem::path{value} / "asr_sdm_log_collector";
  }
  if (const char * value = environmentValue("ROS_HOME")) {
    return std::filesystem::path{value} / "log" / "asr_sdm_log_collector";
  }
  if (const char * value = environmentValue("HOME")) {
    return std::filesystem::path{value} / ".ros" / "log" / "asr_sdm_log_collector";
  }
  return std::filesystem::temp_directory_path() / "asr_sdm_log_collector";
}

/// Creates `directory` and confirms we may write into it. Existing but read-only
/// directories have to be caught here, because create_directories reports success
/// for those and the failure would otherwise surface as a sink exception later.
bool prepareDirectory(const std::filesystem::path & directory, std::string & failure)
{
  std::error_code error;
  std::filesystem::create_directories(directory, error);
  if (error) {
    failure = error.message();
    return false;
  }
  if (::access(directory.c_str(), W_OK) != 0) {
    failure = std::strerror(errno);
    return false;
  }
  return true;
}

std::string runDirectoryName()
{
  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm parts{};
  ::localtime_r(&now, &parts);

  char stamp[32] = {};
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &parts);
  return std::string{stamp} + "_" + std::to_string(::getpid());
}

std::string sanitizeForFileName(const std::string & name)
{
  std::string result;
  result.reserve(name.size());
  for (const char character : name) {
    const bool allowed = std::isalnum(static_cast<unsigned char>(character)) != 0 ||
                         character == '.' || character == '-' || character == '_';
    result.push_back(allowed ? character : '_');
  }
  if (result.empty()) {
    result = "unknown";
  }
  // A leading dot would hide the file and "." / ".." are not usable names.
  if (result.front() == '.') {
    result.front() = '_';
  }
  if (result.size() > 96) {
    result.resize(96);
  }
  return result;
}

void pointLatestSymlinkAt(const std::filesystem::path & run_directory)
{
  const auto link = run_directory.parent_path() / "latest";
  std::error_code error;
  std::filesystem::remove(link, error);
  std::filesystem::create_directory_symlink(run_directory.filename(), link, error);
}

}  // namespace

LogWriter::LogWriter(Config config) : config_{std::move(config)}
{
  // The configured path comes straight out of YAML, so any `~` or `$VAR` in it
  // is still literal text at this point.
  const std::string configured = expandUserPath(config_.log_directory);

  // A deployment path such as /var/log/vehicle is not writable on a development
  // machine, and refusing to start would leave the whole robot without a
  // collector. Fall back instead, and let the caller report that loudly.
  std::vector<std::filesystem::path> candidates;
  if (!configured.empty()) {
    candidates.emplace_back(configured);
  }
  candidates.push_back(derivedBaseDirectory());

  std::string first_failure;
  for (const auto & base : candidates) {
    auto candidate = config_.use_run_subdirectory ? base / runDirectoryName() : base;
    std::string failure;
    if (prepareDirectory(candidate, failure)) {
      directory_ = std::move(candidate);
      break;
    }
    if (first_failure.empty()) {
      first_failure = "cannot use log directory '" + base.string() + "': " + failure;
    }
  }

  if (directory_.empty()) {
    throw std::runtime_error(first_failure);
  }
  if (!first_failure.empty()) {
    directory_warning_ = first_failure + ". Falling back to " + directory_.string();
  }

  if (config_.use_run_subdirectory) {
    pointLatestSymlinkAt(directory_);
  }

  const auto merged_path = directory_ / config_.log_filename;
  merged_sink_ = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
    merged_path.string(), config_.max_file_size_bytes, config_.max_files);
  merged_sink_->set_formatter(makeFormatter(config_.file_pattern));

  if (config_.echo_to_console) {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_formatter(makeFormatter(config_.console_pattern));
    console_sink_ = std::move(console_sink);
  }
}

LogWriter::~LogWriter()
{
  flush();
}

std::unique_ptr<spdlog::formatter> LogWriter::makeFormatter(const std::string & pattern) const
{
  auto formatter = std::make_unique<spdlog::pattern_formatter>();
  formatter->add_flag<OriginPidFlag>('*').add_flag<SourceLocationFlag>('_').set_pattern(pattern);
  return formatter;
}

void LogWriter::write(const LogRecord & record)
{
  const std::lock_guard<std::mutex> guard{mutex_};

  t_origin_pid = record.pid;

  const spdlog::source_loc source{record.file.c_str(), record.line, record.function.c_str()};
  spdlog::details::log_msg message{
    record.time, source, record.logger_name, record.level, record.message};
  message.thread_id = record.thread_id;

  merged_sink_->log(message);
  if (const auto sink = sinkForNode(record.logger_name)) {
    sink->log(message);
  }
  if (console_sink_) {
    console_sink_->log(message);
  }

  ++per_node_counts_[record.logger_name];

  if (record.level >= config_.flush_level) {
    flushLocked();
  }
}

spdlog::sink_ptr LogWriter::sinkForNode(const std::string & node_name)
{
  if (!config_.per_node_files) {
    return nullptr;
  }

  // Keyed by file name so two node names that sanitize alike share one sink
  // instead of two sinks fighting over the same file.
  const std::string file_stem = sanitizeForFileName(node_name);
  const auto existing = node_sinks_.find(file_stem);
  if (existing != node_sinks_.end()) {
    return existing->second;
  }

  if (node_sinks_.size() >= config_.max_tracked_nodes) {
    ++unrouted_nodes_;
    return nullptr;
  }

  const auto path = directory_ / (file_stem + ".log");
  try {
    auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      path.string(), config_.max_file_size_bytes, config_.max_files);
    sink->set_formatter(makeFormatter(config_.file_pattern));
    node_sinks_.emplace(file_stem, sink);
    return sink;
  } catch (const std::exception & error) {
    ++unrouted_nodes_;
    std::cerr << "[asr_sdm_log_collector] cannot open " << path << ": " << error.what()
              << std::endl;
    return nullptr;
  }
}

void LogWriter::flush()
{
  const std::lock_guard<std::mutex> guard{mutex_};
  flushLocked();
}

void LogWriter::flushLocked()
{
  merged_sink_->flush();
  for (auto & entry : node_sinks_) {
    entry.second->flush();
  }
  if (console_sink_) {
    console_sink_->flush();
  }
}

const std::filesystem::path & LogWriter::directory() const
{
  return directory_;
}

const std::string & LogWriter::directoryWarning() const
{
  return directory_warning_;
}

std::map<std::string, uint64_t> LogWriter::perNodeCounts() const
{
  const std::lock_guard<std::mutex> guard{mutex_};
  return per_node_counts_;
}

uint64_t LogWriter::unroutedNodeCount() const
{
  const std::lock_guard<std::mutex> guard{mutex_};
  return unrouted_nodes_;
}

}  // namespace asr_sdm::log
