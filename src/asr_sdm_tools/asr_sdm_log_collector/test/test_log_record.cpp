#include "asr_sdm_log_collector/log_record.hpp"

#include <gtest/gtest.h>
#include <spdlog/details/log_msg.h>
#include <spdlog/pattern_formatter.h>
#include <unistd.h>

#include <chrono>
#include <string>

namespace
{

using asr_sdm::log::LogRecord;

std::chrono::system_clock::time_point microsecondTruncatedNow()
{
  // The wire format carries microseconds, so drop the finer digits before
  // comparing or the round-trip can never be exact.
  return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now());
}

}  // namespace

TEST(LogRecord, SerializeParseRoundTrip)
{
  LogRecord original;
  original.time = microsecondTruncatedNow();
  original.level = spdlog::level::warn;
  original.logger_name = "asr_sdm_hardware";
  original.pid = 4242;
  original.thread_id = 99;
  original.file = "uart2can.cpp";
  original.line = 128;
  original.function = "readFrame";
  original.message = "screw unit 3 stalled";

  const std::string datagram = asr_sdm::log::serializeRecord(original);
  const LogRecord parsed = asr_sdm::log::parseDatagram(datagram.data(), datagram.size(), "unknown");

  EXPECT_EQ(parsed.time, original.time);
  EXPECT_EQ(parsed.level, original.level);
  EXPECT_EQ(parsed.logger_name, original.logger_name);
  EXPECT_EQ(parsed.pid, original.pid);
  EXPECT_EQ(parsed.thread_id, original.thread_id);
  EXPECT_EQ(parsed.file, original.file);
  EXPECT_EQ(parsed.line, original.line);
  EXPECT_EQ(parsed.function, original.function);
  EXPECT_EQ(parsed.message, original.message);
}

/// Guards the actual contract between the two halves of this package: whatever
/// the sending sink's pattern produces must be what the collector can read back.
TEST(LogRecord, ClientPatternIsReadableByCollector)
{
  const auto time = microsecondTruncatedNow();
  const spdlog::source_loc source{"comm_protocol.cpp", 77, "setActuatorCMD"};
  spdlog::details::log_msg message{
    time, source, "asr_sdm_teleop", spdlog::level::err, "joystick timeout"};
  message.thread_id = 31337;

  spdlog::pattern_formatter formatter{asr_sdm::log::wire::pattern()};
  spdlog::memory_buf_t buffer;
  formatter.format(message, buffer);

  ASSERT_TRUE(asr_sdm::log::isStructuredDatagram(buffer.data(), buffer.size()));
  const LogRecord parsed = asr_sdm::log::parseDatagram(buffer.data(), buffer.size(), "unknown");

  EXPECT_EQ(parsed.time, time);
  EXPECT_EQ(parsed.level, spdlog::level::err);
  EXPECT_EQ(parsed.logger_name, "asr_sdm_teleop");
  EXPECT_EQ(parsed.pid, static_cast<int64_t>(::getpid()));
  EXPECT_EQ(parsed.thread_id, 31337u);
  EXPECT_EQ(parsed.file, "comm_protocol.cpp");
  EXPECT_EQ(parsed.line, 77);
  EXPECT_EQ(parsed.function, "setActuatorCMD");
  EXPECT_EQ(parsed.message, "joystick timeout");
}

TEST(LogRecord, RecordWithoutSourceLocationStaysEmpty)
{
  const auto time = microsecondTruncatedNow();
  spdlog::details::log_msg message{time, spdlog::source_loc{}, "plain", spdlog::level::info, "hi"};

  spdlog::pattern_formatter formatter{asr_sdm::log::wire::pattern()};
  spdlog::memory_buf_t buffer;
  formatter.format(message, buffer);

  const LogRecord parsed = asr_sdm::log::parseDatagram(buffer.data(), buffer.size(), "unknown");

  EXPECT_EQ(parsed.line, 0);
  EXPECT_TRUE(parsed.file.empty());
  EXPECT_EQ(parsed.message, "hi");
}

/// A stray sender using a stock spdlog pattern must still be collected rather
/// than discarded, which is why parsing has a fallback path.
TEST(LogRecord, UnstructuredPayloadBecomesMessage)
{
  const std::string payload = "[2026-08-24 08:45:12.001] [info] hand-rolled line\n";
  const LogRecord parsed =
    asr_sdm::log::parseDatagram(payload.data(), payload.size(), "mystery_sender");

  EXPECT_EQ(parsed.logger_name, "mystery_sender");
  EXPECT_EQ(parsed.level, spdlog::level::info);
  EXPECT_EQ(parsed.message, "[2026-08-24 08:45:12.001] [info] hand-rolled line");
}

TEST(LogRecord, MessageMayContainFieldSeparators)
{
  LogRecord original;
  original.time = microsecondTruncatedNow();
  original.logger_name = "asr_sdm_hardware";
  original.message = std::string{"a"} + asr_sdm::log::wire::kFieldSeparator + "b";

  const std::string datagram = asr_sdm::log::serializeRecord(original);
  const LogRecord parsed = asr_sdm::log::parseDatagram(datagram.data(), datagram.size(), "unknown");

  EXPECT_EQ(parsed.message, original.message);
}

TEST(LogRecord, EveryLevelSurvivesTheRoundTrip)
{
  for (int level = spdlog::level::trace; level <= spdlog::level::critical; ++level) {
    LogRecord original;
    original.time = microsecondTruncatedNow();
    original.level = static_cast<spdlog::level::level_enum>(level);
    original.logger_name = "levels";
    original.message = "payload";

    const std::string datagram = asr_sdm::log::serializeRecord(original);
    const LogRecord parsed =
      asr_sdm::log::parseDatagram(datagram.data(), datagram.size(), "unknown");
    EXPECT_EQ(parsed.level, original.level) << "level index " << level;
  }
}
