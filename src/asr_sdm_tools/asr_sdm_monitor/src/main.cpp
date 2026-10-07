#include "asr_sdm_monitor/slint_ui_controller.hpp"

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "asr_sdm_monitor/ros_ui_bridge.hpp"
#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"
#include "asr_sdm_monitor/system_monitor/system_monitor.hpp"

#include <chrono>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    app.setQuitOnLastWindowClosed(false);

    auto window = asr_sdm_monitor_ui::MainWindow::create();

    int ret = 0;
    {
        RosUiBridge bridge;
        const auto node_options = asr_sdm_monitor::system_monitor::makeSystemMonitorNodeOptions();
        auto settings_node = std::make_shared<rclcpp::Node>("asr_sdm_monitor", node_options);
        const auto settings = asr_sdm_monitor::system_monitor::loadSystemMonitorSettings(*settings_node);
        bridge.addHardwareNode(settings_node);

        const std::string hostname = asr_sdm_monitor::system_monitor::normalizedHostname();
        for (const auto &monitor_node : asr_sdm_monitor::system_monitor::createSystemMonitorNodes(
                 settings, hostname, node_options)) {
            bridge.addHardwareNode(monitor_node);
        }
        bridge.startRosExecutors();

        SlintUiController controller(bridge, window);
        // RosUiBridge still schedules work on QTimer. Pump those events from the
        // Slint UI thread so diagnostics, plot flush, and playback keep running.
        slint::Timer qt_pump(std::chrono::milliseconds(33), [&] {
            QCoreApplication::processEvents(QEventLoop::AllEvents, 8);
            controller.sync();
            if (!rclcpp::ok()) {
                slint::quit_event_loop();
            }
        });
        window->run();
    }

    rclcpp::shutdown();
    return ret;
}
