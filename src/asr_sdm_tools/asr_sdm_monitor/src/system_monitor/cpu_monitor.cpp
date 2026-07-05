#include "asr_sdm_monitor/system_monitor/cpu_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>

#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"

namespace asr_sdm_monitor
{
namespace system_monitor
{

namespace
{
diagnostic_msgs::msg::DiagnosticStatus makeInitialStatus()
{
    diagnostic_msgs::msg::DiagnosticStatus stat;
    stat.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    stat.message = "No Data";
    stat.values = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("No Data"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("N/A"),
    };
    return stat;
}
}  // namespace

CpuMonitor::CpuMonitor(
    const std::string & hostname,
    const std::string & diag_hostname,
    const rclcpp::NodeOptions & options)
: Node("cpu_monitor", options)
{
    check_core_temps_ = declare_parameter("check_core_temps", true);
    cpu_load_warn_ = declare_parameter("cpu_load_warn", 0.9);
    cpu_load_error_ = declare_parameter("cpu_load_error", 1.1);
    cpu_load1_warn_ = declare_parameter("cpu_load1_warn", 0.9);
    cpu_load5_warn_ = declare_parameter("cpu_load5_warn", 0.8);
    cpu_temp_warn_ = declare_parameter("cpu_temp_warn", 85.0);
    cpu_temp_error_ = declare_parameter("cpu_temp_error", 90.0);

    num_cores_ = static_cast<int>(std::thread::hardware_concurrency());

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID(hostname);

    temp_stat_ = makeInitialStatus();
    usage_stat_ = makeInitialStatus();

    updater_->add(
        "CPU Temperature (" + diag_hostname + ")",
        this, &CpuMonitor::updateTempStatus);
    updater_->add(
        "CPU Usage (" + diag_hostname + ")",
        this, &CpuMonitor::updateUsageStatus);

    temps_timer_ = create_wall_timer(
        std::chrono::milliseconds(5000),
        [this]() { checkTemps(); });
    usage_timer_ = create_wall_timer(
        std::chrono::milliseconds(5000),
        [this]() { checkUsage(); });

    checkTemps();
    checkUsage();
}

CpuMonitor::CheckResult CpuMonitor::checkCoreTemps()
{
    CheckResult result;
    const auto cmd = runShellCommand("sensors");
    if (cmd.return_code != 0) {
        result.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        result.message = "Core Temperature Error";
        result.values = {
            diagnostic_msgs::msg::KeyValue().set__key("Core Temperature Error").set__value(cmd.stderr_text),
            diagnostic_msgs::msg::KeyValue().set__key("Output").set__value(cmd.stdout_text),
        };
        return result;
    }

    static const std::regex core_pattern(R"(Core [0-9]{1,}:.*)");
    int index = 0;
    for (const auto & line : splitLines(cmd.stdout_text)) {
        if (!std::regex_match(line, core_pattern)) {
            continue;
        }

        const auto tokens = splitWhitespace(line);
        if (tokens.size() < 3) {
            continue;
        }

        std::string tmp = tokens[2];
        if (!tmp.empty() && tmp.back() == '+') {
            tmp.pop_back();
        }
        if (tmp.size() >= 2 && tmp.substr(tmp.size() - 2) == "°C") {
            tmp = tmp.substr(0, tmp.size() - 2);
        } else if (!tmp.empty() && tmp.back() == 'C') {
            tmp.pop_back();
        }

        try {
            const double temp = std::stod(tmp);
            result.values.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Core " + std::to_string(index) + " Temperature")
                    .set__value(std::to_string(temp) + "DegC"));

            if (temp >= cpu_temp_error_) {
                result.level = maxLevel(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
                result.message = "Hot";
            } else if (temp >= cpu_temp_warn_) {
                result.level = maxLevel(result.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
                result.message = "Warm";
            }
        } catch (const std::exception &) {
            result.level = maxLevel(result.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
            result.values.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Core " + std::to_string(index) + " Temperature")
                    .set__value(tmp));
        }
        ++index;
    }

    return result;
}

CpuMonitor::CheckResult CpuMonitor::checkClockSpeed()
{
    CheckResult result;
    const auto cmd = runShellCommand("cat /proc/cpuinfo | grep MHz");
    if (cmd.return_code != 0) {
        result.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        result.message = "Clock speed error";
        result.values = {
            diagnostic_msgs::msg::KeyValue().set__key("Clock speed error").set__value(cmd.stderr_text),
            diagnostic_msgs::msg::KeyValue().set__key("Output").set__value(cmd.stdout_text),
        };
        return result;
    }

    int index = 0;
    for (const auto & line : splitLines(cmd.stdout_text)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        std::string speed = line.substr(colon + 1);
        speed.erase(0, speed.find_first_not_of(" \t"));
        const auto dot = speed.find('.');
        if (dot != std::string::npos) {
            speed = speed.substr(0, dot);
        }

        result.values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Core " + std::to_string(index) + " Clock Speed")
                .set__value(speed + "MHz"));
        ++index;
    }

    return result;
}

std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>> CpuMonitor::checkUptime()
{
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    const auto cmd = runShellCommand("uptime");
    if (cmd.return_code != 0) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("uptime Failed").set__value(cmd.stdout_text));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error", values};
    }

    const auto tokens = splitWhitespace(cmd.stdout_text);
    if (tokens.size() < 3) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("uptime Failed").set__value("Unexpected output"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error", values};
    }

    const int cores = std::max(num_cores_, 1);
    std::string load1_str = tokens[tokens.size() - 3];
    std::string load5_str = tokens[tokens.size() - 2];
    std::string load15_str = tokens[tokens.size() - 1];
    if (!load1_str.empty() && load1_str.back() == ',') {
        load1_str.pop_back();
    }
    if (!load5_str.empty() && load5_str.back() == ',') {
        load5_str.pop_back();
    }

    const double load1 = std::stod(load1_str) / cores;
    const double load5 = std::stod(load5_str) / cores;
    const double load15 = std::stod(load15_str) / cores;

    if (load1 > cpu_load1_warn_ || load5 > cpu_load5_warn_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    values.push_back(
        diagnostic_msgs::msg::KeyValue().set__key("Load Average Status").set__value(uptimeLoadDict(level)));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Load Average (1min)")
            .set__value(std::to_string(load1 * 100.0) + "%"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Load Average (5min)")
            .set__value(std::to_string(load5 * 100.0) + "%"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Load Average (15min)")
            .set__value(std::to_string(load15 * 100.0) + "%"));

    return {level, uptimeLoadDict(level), values};
}

CpuMonitor::CheckResult CpuMonitor::checkMpstat()
{
    CheckResult result;
    const auto cmd = runShellCommand("mpstat -P ALL 1 1");
    if (cmd.return_code != 0) {
        if (!has_warned_mpstat_) {
            RCLCPP_ERROR(
                get_logger(),
                "mpstat failed to run for cpu_monitor. Return code %d.",
                cmd.return_code);
            has_warned_mpstat_ = true;
        }
        result.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        result.message = "Unable to Check CPU Usage";
        result.values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("\"mpstat\" Call Error")
                .set__value(std::to_string(cmd.return_code)));
        return result;
    }

    const auto rows = splitLines(cmd.stdout_text);
    int idle_col = -2;
    if (rows.size() > 2) {
        const auto col_names = splitWhitespace(rows[2]);
        if (col_names.size() > 2 && col_names.back() == "%idle") {
            idle_col = -1;
        }
    }

    int num_cores = 0;
    int cores_loaded = 0;
    for (size_t index = 0; index < rows.size(); ++index) {
        if (index < 3) {
            continue;
        }

        const auto & row = rows[index];
        if (row.find("all") != std::string::npos) {
            continue;
        }

        const auto lst = splitWhitespace(row);
        if (lst.size() < 8) {
            continue;
        }
        if (lst[0].rfind("Average", 0) == 0) {
            continue;
        }

        const int idle_index = idle_col < 0
            ? static_cast<int>(lst.size()) + idle_col
            : idle_col;
        if (idle_index < 0 || idle_index >= static_cast<int>(lst.size())) {
            continue;
        }

        const std::string cpu_name = std::to_string(num_cores);
        const std::string & idle = lst[idle_index];
        const std::string & user = lst[3];
        const std::string & nice = lst[4];
        const std::string & system = lst[5];

        uint8_t core_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        double usage = (std::stod(user) + std::stod(nice)) * 0.01;
        if (usage > 10.0) {
            RCLCPP_WARN(
                get_logger(),
                "Read CPU usage of %f percent. Reverting to previous reading of %f percent",
                usage, usage_old_);
            usage = usage_old_;
        }
        usage_old_ = usage;

        if (usage >= cpu_load_error_) {
            core_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        } else if (usage >= cpu_load_warn_) {
            core_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            ++cores_loaded;
        }

        result.values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Core " + cpu_name + " Status")
                .set__value(mpstatLoadDict(core_level)));
        result.values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Core " + cpu_name + " User").set__value(user + "%"));
        result.values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Core " + cpu_name + " Nice").set__value(nice + "%"));
        result.values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Core " + cpu_name + " System").set__value(system + "%"));
        result.values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Core " + cpu_name + " Idle").set__value(idle + "%"));

        ++num_cores;
    }

    if (num_cores - cores_loaded <= 2 && num_cores > 2) {
        result.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    if (num_cores_ == 0) {
        num_cores_ = num_cores;
    }

    if (num_cores_ != num_cores) {
        if (!has_error_core_count_) {
            RCLCPP_ERROR(
                get_logger(),
                "Error checking number of cores. Expected %d got %d. "
                "Computer may have not booted properly.",
                num_cores_, num_cores);
            has_error_core_count_ = true;
        }
        result.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        result.message = "Incorrect number of CPU cores";
        return result;
    }

    result.message = mpstatLoadDict(result.level);
    return result;
}

void CpuMonitor::checkTemps()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    std::set<std::string> diag_msgs;
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    if (check_core_temps_) {
        const auto core = checkCoreTemps();
        diag_vals.insert(diag_vals.end(), core.values.begin(), core.values.end());
        if (!core.message.empty()) {
            diag_msgs.insert(core.message);
        }
        diag_level = maxLevel(diag_level, core.level);
    }

    std::string message;
    if (!diag_msgs.empty()) {
        message = *diag_msgs.begin();
        for (auto it = std::next(diag_msgs.begin()); it != diag_msgs.end(); ++it) {
            message += ", " + *it;
        }
    } else {
        message = statDict(diag_level);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_temp_time_ = get_clock()->now();
    temp_stat_.level = diag_level;
    temp_stat_.message = message;
    temp_stat_.values = std::move(diag_vals);
}

void CpuMonitor::checkUsage()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    std::set<std::string> diag_msgs;
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    const auto clock = checkClockSpeed();
    diag_vals.insert(diag_vals.end(), clock.values.begin(), clock.values.end());
    if (!clock.message.empty()) {
        diag_msgs.insert(clock.message);
    }
    diag_level = maxLevel(diag_level, clock.level);

    const auto mpstat = checkMpstat();
    diag_vals.insert(diag_vals.end(), mpstat.values.begin(), mpstat.values.end());
    if (mpstat.level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        diag_msgs.insert(mpstat.message);
    }
    diag_level = maxLevel(diag_level, mpstat.level);

    const auto [uptime_level, up_msg, up_vals] = checkUptime();
    diag_vals.insert(diag_vals.end(), up_vals.begin(), up_vals.end());
    if (uptime_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        diag_msgs.insert(up_msg);
    }
    diag_level = maxLevel(diag_level, uptime_level);

    std::string usage_msg;
    if (!diag_msgs.empty() && diag_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        usage_msg = *diag_msgs.begin();
        for (auto it = std::next(diag_msgs.begin()); it != diag_msgs.end(); ++it) {
            usage_msg += ", " + *it;
        }
    } else {
        usage_msg = statDict(diag_level);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_usage_time_ = get_clock()->now();
    usage_stat_.level = diag_level;
    usage_stat_.values = std::move(diag_vals);
    usage_stat_.message = usage_msg;
}

void CpuMonitor::updateTempStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto temp_copy = temp_stat_;
    updateStatusStale(temp_copy, *get_clock(), last_temp_time_);
    stat.summary(temp_copy.level, temp_copy.message);
    for (const auto & value : temp_copy.values) {
        stat.add(value.key, value.value);
    }
}

void CpuMonitor::updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto usage_copy = usage_stat_;
    updateStatusStale(usage_copy, *get_clock(), last_usage_time_);
    stat.summary(usage_copy.level, usage_copy.message);
    for (const auto & value : usage_copy.values) {
        stat.add(value.key, value.value);
    }
}

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
