#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"

#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <memory>
#include <sstream>
#include <sys/wait.h>
#include <unistd.h>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

namespace
{

// Using decltype(&pclose) as std::unique_ptr's deleter triggers
// -Wignored-attributes with newer GCC/glibc combinations because pclose carries
// function attributes. A small function object keeps the same RAII behaviour
// without placing the attributed function type in a template argument.
struct PipeCloser
{
    void operator()(FILE * pipe) const noexcept
    {
        if (pipe != nullptr) {
            (void)pclose(pipe);
        }
    }
};

std::string shellQuote(const std::string & argument)
{
    std::string quoted;
    quoted.reserve(argument.size() + 2);
    quoted.push_back('\'');
    for (const char ch : argument) {
        if (ch == '\'') {
            quoted += "'\\''";
        } else {
            quoted.push_back(ch);
        }
    }
    quoted.push_back('\'');
    return quoted;
}

}  // namespace

CommandResult runShellCommand(const std::string & command)
{
    CommandResult result;
    std::array<char, 4096> buffer{};

    // The parsers expect stable English output and a dot decimal separator.
    // Redirect stderr to stdout because popen() exposes only one stream.
    const std::string wrapped = "LC_ALL=C; export LC_ALL; " + command + " 2>&1";
    std::unique_ptr<FILE, PipeCloser> pipe{popen(wrapped.c_str(), "r")};
    if (!pipe) {
        result.stderr_text = std::string("Failed to open pipe: ") + std::strerror(errno);
        return result;
    }

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result.stdout_text += buffer.data();
    }

    FILE * const raw_pipe = pipe.release();
    const int status = pclose(raw_pipe);
    if (status == -1) {
        result.stderr_text = std::string("Failed to close pipe: ") + std::strerror(errno);
        return result;
    }

    if (WIFEXITED(status)) {
        result.return_code = WEXITSTATUS(status);
    } else if (WIFSIGNALED(status)) {
        result.return_code = 128 + WTERMSIG(status);
    }

    // stderr was redirected into stdout. Preserve the captured diagnostic text
    // in stderr_text as well when the command failed, so callers can report it.
    if (result.return_code != 0) {
        result.stderr_text = result.stdout_text;
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
        command += shellQuote(args[i]);
    }
    return runShellCommand(command);
}

std::string normalizedHostname()
{
    std::array<char, 256> hostname{};
    if (gethostname(hostname.data(), hostname.size()) != 0) {
        return "unknown";
    }
    hostname.back() = '\0';

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
        std::ostringstream elapsed;
        elapsed << std::fixed << std::setprecision(3) << time_since_update.seconds() << 's';
        stat.values[0].key = "Update Status";
        stat.values[0].value = stale_status;
        stat.values[1].key = "Time Since Last Update";
        stat.values[1].value = elapsed.str();
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
