#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

class CpuMonitor : public rclcpp::Node
{
public:
    CpuMonitor(
        const std::string & hostname,
        const std::string & diag_hostname,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    struct CheckResult
    {
        uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        std::string message;
        std::vector<diagnostic_msgs::msg::KeyValue> values;
    };

    void checkTemps();
    void checkUsage();
    void updateTempStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
    void updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);

    CheckResult checkCoreTemps();
    CheckResult checkClockSpeed();
    CheckResult checkMpstat();
    std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>> checkUptime();

    std::shared_ptr<diagnostic_updater::Updater> updater_;
    std::mutex mutex_;

    bool check_core_temps_ = true;
    double cpu_load_warn_ = 0.9;
    double cpu_load_error_ = 1.0;
    double cpu_load1_warn_ = 0.9;
    double cpu_load5_warn_ = 0.8;
    double cpu_temp_warn_ = 85.0;
    double cpu_temp_error_ = 90.0;

    int num_cores_ = 0;
    diagnostic_msgs::msg::DiagnosticStatus temp_stat_;
    diagnostic_msgs::msg::DiagnosticStatus usage_stat_;
    rclcpp::Time last_temp_time_;
    rclcpp::Time last_usage_time_;
    rclcpp::TimerBase::SharedPtr temps_timer_;
    rclcpp::TimerBase::SharedPtr usage_timer_;

    double usage_old_ = 0.0;
    bool has_warned_mpstat_ = false;
    bool has_error_core_count_ = false;
};

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
