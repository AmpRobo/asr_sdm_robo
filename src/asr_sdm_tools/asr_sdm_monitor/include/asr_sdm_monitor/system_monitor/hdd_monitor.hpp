#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <QJsonObject>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

class HddMonitor : public rclcpp::Node
{
public:
    HddMonitor(
        const std::string & hostname,
        const std::string & diag_hostname,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void checkTemps();
    void checkDiskUsage();
    void updateTempStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
    void updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
    void extractTempInputs(
        const QJsonObject & object,
        const std::string & prefix,
        std::vector<std::string> & paths,
        std::vector<double> & temps) const;

    std::shared_ptr<diagnostic_updater::Updater> updater_;
    std::mutex mutex_;

    bool no_temp_ = false;
    bool no_temp_warn_ = false;
    double hdd_level_warn_ = 0.95;
    double hdd_level_error_ = 0.99;
    double hdd_temp_warn_ = 55.0;
    double hdd_temp_error_ = 70.0;

    diagnostic_msgs::msg::DiagnosticStatus temp_stat_;
    diagnostic_msgs::msg::DiagnosticStatus usage_stat_;
    rclcpp::Time last_temp_time_;
    rclcpp::Time last_usage_time_;
    rclcpp::TimerBase::SharedPtr temp_timer_;
    rclcpp::TimerBase::SharedPtr usage_timer_;
    bool has_warned_sensors_ = false;
};

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
