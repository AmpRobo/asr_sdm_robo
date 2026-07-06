#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

struct CommandResult
{
    int return_code = -1;
    std::string stdout_text;
    std::string stderr_text;
};

CommandResult runShellCommand(const std::string & command);
CommandResult runCommand(const std::vector<std::string> & args);

std::string normalizedHostname();
uint8_t maxLevel(uint8_t a, uint8_t b);

const char * statDict(uint8_t level);
const char * mpstatLoadDict(uint8_t level);
const char * uptimeLoadDict(uint8_t level);
const char * memDict(uint8_t level);
const char * usageDict(uint8_t level);
const char * tempDict(uint8_t level);
const char * netDict(uint8_t level);

void updateStatusStale(
    diagnostic_msgs::msg::DiagnosticStatus & stat,
    rclcpp::Clock & clock,
    const rclcpp::Time & last_update_time);

std::vector<std::string> splitWhitespace(const std::string & text);
std::vector<std::string> splitLines(const std::string & text);

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
