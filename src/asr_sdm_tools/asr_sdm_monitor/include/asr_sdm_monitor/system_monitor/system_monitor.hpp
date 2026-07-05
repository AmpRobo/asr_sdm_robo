#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace asr_sdm_monitor
{
namespace system_monitor
{

struct SystemMonitorSettings
{
    bool enable_cpu_monitor = true;
    bool enable_hdd_monitor = true;
    bool enable_mem_monitor = true;
    bool enable_net_monitor = true;
    std::string diag_hostname;
};

SystemMonitorSettings loadSystemMonitorSettings(rclcpp::Node & node);

std::string defaultSystemMonitorConfigPath();
rclcpp::NodeOptions makeSystemMonitorNodeOptions();

std::vector<rclcpp::Node::SharedPtr> createSystemMonitorNodes(
    const SystemMonitorSettings & settings,
    const std::string & hostname,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
