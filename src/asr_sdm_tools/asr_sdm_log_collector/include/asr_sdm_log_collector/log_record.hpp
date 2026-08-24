#ifndef ASR_SDM_LOG_COLLECTOR__LOG_RECORD_HPP_
#define ASR_SDM_LOG_COLLECTOR__LOG_RECORD_HPP_

#include <spdlog/common.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

namespace asr_sdm::log
{

/// One log line as it crossed the wire, with the origin's identity preserved.
struct LogRecord
{
  std::chrono::system_clock::time_point time{};
  spdlog::level::level_enum level{spdlog::level::info};
  std::string logger_name;
  int64_t pid{0};
  std::size_t thread_id{0};
  std::string file;
  int line{0};
  std::string function;
  std::string message;
};

/// Datagram layout shared by the sending sink and the collector.
///
/// A record is a single UDP datagram of separator-delimited fields:
///   ASR1 <US> epoch_seconds.microseconds <US> level <US> logger <US> pid
///        <US> tid <US> file <US> line <US> function <US> message
///
/// The separator is ASCII Unit Separator, which never appears in log text, so
/// the message may keep its own newlines and stay the trailing field.
namespace wire
{

constexpr char kFieldSeparator = '\x1f';
constexpr char kMagic[] = "ASR1";
constexpr std::size_t kFieldCount = 10;

/// spdlog pattern producing a datagram that `parseDatagram` reads back exactly.
/// Set this on the sending sink; it is not meant to be human readable.
const std::string & pattern();

}  // namespace wire

/// True when the payload carries the structured header this package emits.
bool isStructuredDatagram(const char * data, std::size_t size);

/// Decodes one datagram. Never throws and never rejects input: a payload that
/// is not in the structured format becomes a record whose message is the whole
/// payload, so plain `spdlog` sinks and stray senders are still collected
/// rather than silently dropped.
LogRecord parseDatagram(
  const char * data, std::size_t size, const std::string & fallback_logger_name);

/// Renders a record back into a datagram. Only needed by tests and by senders
/// that build records by hand instead of going through spdlog.
std::string serializeRecord(const LogRecord & record);

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__LOG_RECORD_HPP_
