#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"

#include <array>
#include <cstdio>
#include <cstring>
#include <memory>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

CommandResult runShellCommand(const std::string & command)
{
    CommandResult result;
    std::array<char, 4096> buffer{};

    const std::string wrapped = command + " 2>&1";
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(wrapped.c_str(), "r"), pclose);
    if (!pipe) {
        result.stderr_text = "Failed to open pipe";
        return result;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result.stdout_text += buffer.data();
    }

    const int status = pclose(pipe.release());
    if (WIFEXITED(status)) {
        result.return_code = WEXITSTATUS(status);
    }
    return result;
}

CommandResult runCommand(const std::vector<std::string> & args)
{
    if (args.empty()) {
        return {};
    }

    std::string command;
    for (size_t i = 0; i < args.size(); ++i) {
        if (i > 0) {
            command += ' ';
        }
        command += args[i];
    }
    return runShellCommand(command);
}

std::string normalizedHostname()
{
    std::array<char, 256> hostname{};
    if (gethostname(hostname.data(), hostname.size()) != 0) {
        return "unknown";
    }

    std::string name(hostname.data());
    for (char & ch : name) {
        if (ch == '-') {
            ch = '_';
        }
    }
    return name;
}

uint8_t maxLevel(uint8_t a, uint8_t b)
{
    return a > b ? a : b;
}

const char * statDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "Warning";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Error";
        default:
            return "OK";
    }
}

const char * mpstatLoadDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "High Load";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Error";
        default:
            return "OK";
    }
}

const char * uptimeLoadDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "High Load";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Very High Load";
        default:
            return "OK";
    }
}

const char * memDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "Low Memory";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Very Low Memory";
        default:
            return "OK";
    }
}

const char * usageDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "Low Disk Space";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Very Low Disk Space";
        default:
            return "OK";
    }
}

const char * tempDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "Hot";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Critical Hot";
        default:
            return "OK";
    }
}

const char * netDict(uint8_t level)
{
    switch (level) {
        case diagnostic_msgs::msg::DiagnosticStatus::WARN:
            return "High Network Usage";
        case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
            return "Network Down";
        default:
            return "OK";
    }
}

void updateStatusStale(
    diagnostic_msgs::msg::DiagnosticStatus & stat,
    rclcpp::Clock & clock,
    const rclcpp::Time & last_update_time)
{
    const rclcpp::Duration time_since_update = clock.now() - last_update_time;

    std::string stale_status = "OK";
    if (time_since_update > rclcpp::Duration::from_seconds(20.0) &&
        time_since_update <= rclcpp::Duration::from_seconds(35.0))
    {
        stale_status = "Lagging";
        if (stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
            stat.message = stale_status;
        } else if (stat.message.find(stale_status) == std::string::npos) {
            stat.message = stat.message + ", " + stale_status;
        }
        stat.level = maxLevel(stat.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    } else if (time_since_update > rclcpp::Duration::from_seconds(35.0)) {
        stale_status = "Stale";
        if (stat.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
            stat.message = stale_status;
        } else if (stat.message.find(stale_status) == std::string::npos) {
            stat.message = stat.message + ", " + stale_status;
        }
        stat.level = maxLevel(stat.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    }

    if (stat.values.size() >= 2) {
        stat.values[0].key = "Update Status";
        stat.values[0].value = stale_status;
        stat.values[1].key = "Time Since Last Update";
        stat.values[1].value = std::to_string(time_since_update.seconds()) + "." +
            std::to_string(time_since_update.nanoseconds() % 1000000000) + "s";
    }
}

std::vector<std::string> splitWhitespace(const std::string & text)
{
    std::vector<std::string> tokens;
    std::istringstream stream(text);
    std::string token;
    while (stream >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

std::vector<std::string> splitLines(const std::string & text)
{
    std::vector<std::string> lines;
    std::istringstream stream(text);
    std::string line;
    while (std::getline(stream, line)) {
        lines.push_back(line);
    }
    return lines;
}

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
