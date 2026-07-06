#include "system_monitor/mem_monitor.hpp"

#include <set>

#include "system_monitor/monitor_utils.hpp"

namespace asr_sdm_monitor
{
namespace system_monitor
{

MemMonitor::MemMonitor(
    const std::string & hostname,
    const std::string & diag_hostname,
    const rclcpp::NodeOptions & options)
: Node("mem_monitor", options)
{
    mem_level_warn_ = declare_parameter("mem_level_warn", 0.95);
    mem_level_error_ = declare_parameter("mem_level_error", 0.99);

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID(hostname);

    usage_stat_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    usage_stat_.message = "No Data";
    usage_stat_.values = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("No Data"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("N/A"),
    };

    updater_->add(
        "Memory Usage (" + diag_hostname + ")",
        this, &MemMonitor::updateUsageStatus);

    usage_timer_ = create_wall_timer(
        std::chrono::milliseconds(5000),
        [this]() { checkUsage(); });

    checkUsage();
}

std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>>
MemMonitor::checkMemory()
{
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    const auto cmd = runShellCommand("free -tm");
    if (cmd.return_code != 0) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("\"free -tm\" Call Error")
                .set__value(std::to_string(cmd.return_code)));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Error", values};
    }

    const auto rows = splitLines(cmd.stdout_text);
    if (rows.size() < 4) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Memory Usage Check Error")
                .set__value("Unexpected free output"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Memory Usage Check Error", values};
    }

    const auto physical = splitWhitespace(rows[1]);
    const auto swap = splitWhitespace(rows[2]);
    const auto total = splitWhitespace(rows[3]);
    if (physical.size() < 4 || swap.size() < 4 || total.size() < 4) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Memory Usage Check Error")
                .set__value("Malformed free output"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Memory Usage Check Error", values};
    }

    const double mem_usage = std::stod(physical[2]) / std::stod(physical[1]);
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (mem_usage >= mem_level_error_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else if (mem_usage >= mem_level_warn_) {
        level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    values.push_back(
        diagnostic_msgs::msg::KeyValue().set__key("Memory Status").set__value(memDict(level)));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Total Memory (Physical)")
            .set__value(physical[1] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Used Memory (Physical)")
            .set__value(physical[2] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Free Memory (Physical)")
            .set__value(physical[3] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Total Memory (Swap)")
            .set__value(swap[1] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Used Memory (Swap)")
            .set__value(swap[2] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Free Memory (Swap)")
            .set__value(swap[3] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Total Memory")
            .set__value(total[1] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Used Memory")
            .set__value(total[2] + "M"));
    values.push_back(
        diagnostic_msgs::msg::KeyValue()
            .set__key("Free Memory")
            .set__value(total[3] + "M"));

    return {level, memDict(level), values};
}

void MemMonitor::checkUsage()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    std::set<std::string> diag_msgs;
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    const auto [mem_level, mem_msg, mem_vals] = checkMemory();
    diag_vals.insert(diag_vals.end(), mem_vals.begin(), mem_vals.end());
    if (mem_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        diag_msgs.insert(mem_msg);
    }
    diag_level = maxLevel(diag_level, mem_level);

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

void MemMonitor::updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
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
