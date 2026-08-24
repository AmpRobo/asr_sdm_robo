#include "asr_sdm_log_collector/log_record.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cstdio>
#include <cstring>
#include <string_view>

namespace asr_sdm::log
{

namespace
{

template <typename T>
bool parseInteger(std::string_view text, T & value)
{
  if (text.empty()) {
    return false;
  }
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}

/// spdlog terminates every formatted line with its end-of-line string. Removing
/// exactly one terminator keeps messages that intentionally end in blank lines.
std::string_view stripOneEol(std::string_view text)
{
  if (!text.empty() && text.back() == '\n') {
    text.remove_suffix(1);
  }
  if (!text.empty() && text.back() == '\r') {
    text.remove_suffix(1);
  }
  return text;
}

std::chrono::system_clock::time_point parseTimestamp(std::string_view field)
{
  const auto dot = field.find('.');
  const std::string_view seconds_text =
    dot == std::string_view::npos ? field : field.substr(0, dot);

  int64_t seconds = 0;
  if (!parseInteger(seconds_text, seconds) || seconds < 0) {
    return std::chrono::system_clock::now();
  }

  // Pad or truncate the fraction so any sub-second precision maps to microseconds.
  std::array<char, 7> digits{'0', '0', '0', '0', '0', '0', '\0'};
  if (dot != std::string_view::npos) {
    const std::string_view fraction = field.substr(dot + 1);
    const std::size_t count = std::min<std::size_t>(fraction.size(), 6);
    std::memcpy(digits.data(), fraction.data(), count);
  }

  int64_t microseconds = 0;
  if (!parseInteger(std::string_view{digits.data(), 6}, microseconds)) {
    microseconds = 0;
  }

  return std::chrono::system_clock::time_point{
    std::chrono::seconds{seconds} + std::chrono::microseconds{microseconds}};
}

spdlog::level::level_enum parseLevel(std::string_view field)
{
  if (field.empty()) {
    return spdlog::level::info;
  }
  const std::string name{field};
  const auto level = spdlog::level::from_str(name);
  // from_str reports unknown names as `off`, which would hide the record.
  if (level == spdlog::level::off && name != "off") {
    return spdlog::level::info;
  }
  return level;
}

bool splitFields(std::string_view payload, std::array<std::string_view, wire::kFieldCount> & fields)
{
  std::size_t start = 0;
  for (std::size_t index = 0; index + 1 < wire::kFieldCount; ++index) {
    const auto separator = payload.find(wire::kFieldSeparator, start);
    if (separator == std::string_view::npos) {
      return false;
    }
    fields[index] = payload.substr(start, separator - start);
    start = separator + 1;
  }
  // The message is last so it may contain separators of its own.
  fields[wire::kFieldCount - 1] = payload.substr(start);
  return true;
}

}  // namespace

const std::string & wire::pattern()
{
  static const std::string kPattern =
    std::string{kMagic} + kFieldSeparator + "%E.%f" + kFieldSeparator + "%l" + kFieldSeparator +
    "%n" + kFieldSeparator + "%P" + kFieldSeparator + "%t" + kFieldSeparator + "%s" +
    kFieldSeparator + "%#" + kFieldSeparator + "%!" + kFieldSeparator + "%v";
  return kPattern;
}

bool isStructuredDatagram(const char * data, std::size_t size)
{
  constexpr std::size_t kMagicLength = sizeof(wire::kMagic) - 1;
  if (data == nullptr || size < kMagicLength + 1) {
    return false;
  }
  return std::memcmp(data, wire::kMagic, kMagicLength) == 0 &&
         data[kMagicLength] == wire::kFieldSeparator;
}

LogRecord parseDatagram(
  const char * data, std::size_t size, const std::string & fallback_logger_name)
{
  LogRecord record;
  record.logger_name = fallback_logger_name;

  const std::string_view payload{data == nullptr ? "" : data, data == nullptr ? 0 : size};

  std::array<std::string_view, wire::kFieldCount> fields{};
  if (!isStructuredDatagram(data, size) || !splitFields(payload, fields)) {
    record.time = std::chrono::system_clock::now();
    record.message = std::string{stripOneEol(payload)};
    return record;
  }

  record.time = parseTimestamp(fields[1]);
  record.level = parseLevel(fields[2]);
  if (!fields[3].empty()) {
    record.logger_name = std::string{fields[3]};
  }
  if (!parseInteger(fields[4], record.pid)) {
    record.pid = 0;
  }
  if (!parseInteger(fields[5], record.thread_id)) {
    record.thread_id = 0;
  }
  record.file = std::string{fields[6]};
  if (!parseInteger(fields[7], record.line)) {
    record.line = 0;
  }
  record.function = std::string{fields[8]};
  record.message = std::string{stripOneEol(fields[9])};

  return record;
}

std::string serializeRecord(const LogRecord & record)
{
  const auto since_epoch = record.time.time_since_epoch();
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(since_epoch);
  const auto microseconds =
    std::chrono::duration_cast<std::chrono::microseconds>(since_epoch - seconds);

  std::array<char, 8> fraction{};
  std::snprintf(fraction.data(), fraction.size(), "%06ld", static_cast<long>(microseconds.count()));

  const auto level_name = spdlog::level::to_string_view(record.level);

  std::string datagram;
  datagram.reserve(record.message.size() + record.logger_name.size() + 96);
  datagram.append(wire::kMagic);
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(std::to_string(seconds.count()));
  datagram.push_back('.');
  datagram.append(fraction.data());
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(level_name.data(), level_name.size());
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(record.logger_name);
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(std::to_string(record.pid));
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(std::to_string(record.thread_id));
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(record.file);
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(record.line == 0 ? "" : std::to_string(record.line));
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(record.function);
  datagram.push_back(wire::kFieldSeparator);
  datagram.append(record.message);
  datagram.push_back('\n');

  return datagram;
}

}  // namespace asr_sdm::log
