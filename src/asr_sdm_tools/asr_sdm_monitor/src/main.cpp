#include <QApplication>
#include <QQmlApplicationEngine>
#include <QUrl>
#include <qqml.h>

#include <rclcpp/rclcpp.hpp>
#include "asr_sdm_monitor/ros_ui_bridge.hpp"
#include "asr_sdm_monitor/ros_video_image_provider.hpp"
#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"
#include "asr_sdm_monitor/system_monitor/system_monitor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    RosUiBridge bridge;

    const auto node_options =
        asr_sdm_monitor::system_monitor::makeSystemMonitorNodeOptions();
    auto settings_node = std::make_shared<rclcpp::Node>("asr_sdm_monitor", node_options);
    const auto settings =
        asr_sdm_monitor::system_monitor::loadSystemMonitorSettings(*settings_node);
    bridge.addNode(settings_node);

    const std::string hostname = asr_sdm_monitor::system_monitor::normalizedHostname();
    for (const auto & monitor_node :
        asr_sdm_monitor::system_monitor::createSystemMonitorNodes(
            settings, hostname, node_options))
    {
        bridge.addNode(monitor_node);
    }

    bridge.startRosExecutor();

    qmlRegisterSingletonInstance("RosUi", 1, 0, "RosUi", &bridge);

    QQmlApplicationEngine engine;
    engine.addImageProvider(QStringLiteral("rosvideo"), new RosVideoImageProvider(&bridge));
    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/AsrSdmMonitor/qml/Main.qml")));

    if (engine.rootObjects().isEmpty()) {
        rclcpp::shutdown();
        return -1;
    }

    const int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
