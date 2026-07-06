#include <QApplication>
#include <QQmlApplicationEngine>
#include <QUrl>
#include <qqml.h>

#include <rclcpp/rclcpp.hpp>
#include "ros_ui_bridge.hpp"
#include "ros_video_image_provider.hpp"
#include "system_monitor/monitor_utils.hpp"
#include "system_monitor/system_monitor.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    int ret = -1;
    {
        // Thread 1 is the Qt GUI thread and owns QML, plot rendering and the
        // final video display. Plot/Record ROS callbacks run in a separate
        // executor thread owned by RosUiBridge.
        RosUiBridge bridge;

        const auto node_options =
            asr_sdm_monitor::system_monitor::makeSystemMonitorNodeOptions();
        auto settings_node = std::make_shared<rclcpp::Node>("asr_sdm_monitor", node_options);
        const auto settings =
            asr_sdm_monitor::system_monitor::loadSystemMonitorSettings(*settings_node);
        bridge.addHardwareNode(settings_node);

        const std::string hostname = asr_sdm_monitor::system_monitor::normalizedHostname();
        for (const auto & monitor_node :
            asr_sdm_monitor::system_monitor::createSystemMonitorNodes(
                settings, hostname, node_options))
        {
            bridge.addHardwareNode(monitor_node);
        }

        bridge.startRosExecutors();

        qmlRegisterSingletonInstance("RosUi", 1, 0, "RosUi", &bridge);

        QQmlApplicationEngine engine;
        engine.addImageProvider(QStringLiteral("rosvideo"), new RosVideoImageProvider(&bridge));
        engine.load(QUrl(QStringLiteral("qrc:/qt/qml/AsrSdmMonitor/qml/Main.qml")));

        if (!engine.rootObjects().isEmpty()) {
            ret = app.exec();
        }
    }

    // RosUiBridge stops and joins all ROS worker threads before ROS shutdown.
    rclcpp::shutdown();
    return ret;
}
