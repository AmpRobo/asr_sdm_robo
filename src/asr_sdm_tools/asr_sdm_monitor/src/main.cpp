#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QUrl>
#include <qqml.h>

#include <rclcpp/rclcpp.hpp>
#include "asr_sdm_monitor/ros_ui_bridge.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QGuiApplication app(argc, argv);

    RosUiBridge bridge;
    qmlRegisterSingletonInstance("RosUi", 1, 0, "RosUi", &bridge);

    QQmlApplicationEngine engine;
    engine.load(QUrl(QStringLiteral("qrc:/qt/qml/AsrSdmMonitor/qml/Main.qml")));

    if (engine.rootObjects().isEmpty()) {
        rclcpp::shutdown();
        return -1;
    }

    const int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
