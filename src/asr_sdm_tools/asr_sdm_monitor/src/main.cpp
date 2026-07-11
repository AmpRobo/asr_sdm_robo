#include <QApplication>
#include <QQmlApplicationEngine>
#include <QUrl>
#include <QTimer>
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

    // rclcpp handles SIGINT and invalidates its context. Qt has an independent
    // event loop, so explicitly leave it when ROS shuts down (Ctrl+C).
    QTimer ros_shutdown_watcher;
    ros_shutdown_watcher.setInterval(50);
    QObject::connect(&ros_shutdown_watcher, &QTimer::timeout, &app, [&app]()
    {
        if (!rclcpp::ok()) {
            app.quit();
        }
    });
    ros_shutdown_watcher.start();

    int ret = -1;
    {
        // Thread 1 is the Qt GUI thread and owns QML, plot rendering and the
        // final video display. Plot/Record ROS callbacks run in a separate
        // executor thread owned by RosUiBridge.
        RosUiBridge bridge;
        QObject::connect(&app, &QCoreApplication::aboutToQuit,
                         &bridge, &RosUiBridge::shutdown,
                         Qt::DirectConnection);

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
