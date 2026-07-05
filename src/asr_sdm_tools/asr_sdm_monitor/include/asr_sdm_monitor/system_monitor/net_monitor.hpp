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

class NetMonitor : public rclcpp::Node
{
public:
    NetMonitor(
        const std::string & hostname,
        const std::string & diag_hostname,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void checkUsage();
    void updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
    std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>> checkNetwork();
    std::pair<int, std::string> readSysNetStat(const std::string & iface, const std::string & stat_name);
    std::pair<int, std::string> readSysNet(const std::string & iface, const std::string & field);

    std::shared_ptr<diagnostic_updater::Updater> updater_;
    std::mutex mutex_;

    double net_level_warn_ = 0.95;
    double net_capacity_ = 128.0;

    diagnostic_msgs::msg::DiagnosticStatus usage_stat_;
    rclcpp::Time last_usage_time_;
    rclcpp::TimerBase::SharedPtr usage_timer_;
};

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
