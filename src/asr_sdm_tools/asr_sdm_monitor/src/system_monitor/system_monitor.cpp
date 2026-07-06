#include "system_monitor/system_monitor.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "system_monitor/cpu_monitor.hpp"
#include "system_monitor/hdd_monitor.hpp"
#include "system_monitor/mem_monitor.hpp"
#include "system_monitor/monitor_utils.hpp"
#include "system_monitor/net_monitor.hpp"

namespace asr_sdm_monitor
{
namespace system_monitor
{

SystemMonitorSettings loadSystemMonitorSettings(rclcpp::Node & node)
{
    SystemMonitorSettings settings;
    settings.enable_cpu_monitor = node.declare_parameter("enable_cpu_monitor", true);
    settings.enable_hdd_monitor = node.declare_parameter("enable_hdd_monitor", true);
    settings.enable_mem_monitor = node.declare_parameter("enable_mem_monitor", true);
    settings.enable_net_monitor = node.declare_parameter("enable_net_monitor", true);
    settings.diag_hostname = node.declare_parameter("diag_hostname", normalizedHostname());
    return settings;
}

std::string defaultSystemMonitorConfigPath()
{
    try {
        return ament_index_cpp::get_package_share_directory("asr_sdm_monitor") +
            "/config/system_monitor.yaml";
    } catch (const std::exception &) {
        return {};
    }
}

rclcpp::NodeOptions makeSystemMonitorNodeOptions()
{
    rclcpp::NodeOptions options;
    const std::string config_path = defaultSystemMonitorConfigPath();
    if (!config_path.empty()) {
        options.arguments({"--ros-args", "--params-file", config_path});
    }
    return options;
}

std::vector<rclcpp::Node::SharedPtr> createSystemMonitorNodes(
    const SystemMonitorSettings & settings,
    const std::string & hostname,
    const rclcpp::NodeOptions & node_options)
{
    std::vector<rclcpp::Node::SharedPtr> nodes;
    const std::string diag_hostname = settings.diag_hostname.empty()
        ? normalizedHostname()
        : settings.diag_hostname;

    if (settings.enable_cpu_monitor) {
        nodes.push_back(std::make_shared<CpuMonitor>(hostname, diag_hostname, node_options));
    }
    if (settings.enable_hdd_monitor) {
        nodes.push_back(std::make_shared<HddMonitor>(hostname, diag_hostname, node_options));
    }
    if (settings.enable_mem_monitor) {
        nodes.push_back(std::make_shared<MemMonitor>(hostname, diag_hostname, node_options));
    }
    if (settings.enable_net_monitor) {
        nodes.push_back(std::make_shared<NetMonitor>(hostname, diag_hostname, node_options));
    }

    return nodes;
}

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
