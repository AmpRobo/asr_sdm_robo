#include "asr_sdm_monitor/ros_ui_bridge.hpp"
#include "asr_sdm_monitor/ros_executor_manager.hpp"

#include <QTimer>

#include <chrono>

namespace
{
constexpr int kPlotUiUpdateIntervalMs = 50;
constexpr int kRecordedPlaybackUpdateIntervalMs = 50;
}

RosUiBridge::RosUiBridge(QObject *parent)
    : QObject(parent),
      ros_status_("Waiting for /diagnostics and ROS topics ...")
{
    playback_timer_ = new QTimer(this);
    playback_timer_->setInterval(kRecordedPlaybackUpdateIntervalMs);
    connect(playback_timer_, &QTimer::timeout, this, &RosUiBridge::playbackTick);

    plot_update_timer_ = new QTimer(this);
    plot_update_timer_->setInterval(kPlotUiUpdateIntervalMs);
    connect(plot_update_timer_, &QTimer::timeout, this, &RosUiBridge::flushLivePlotSamples);
    plot_update_timer_->start();

    node_ = std::make_shared<rclcpp::Node>("diagnostics_qml_ui_node");
    gui_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    gui_executor_->add_node(node_);

    // Thread 1 services only lightweight diagnostics and ROS graph discovery
    // in short bounded slices. QML, plot rendering and final video display stay
    // owned by the Qt GUI thread.
    gui_ros_spin_timer_ = new QTimer(this);
    gui_ros_spin_timer_->setInterval(10);
    connect(gui_ros_spin_timer_, &QTimer::timeout, this, [this]()
    {
        if (gui_executor_ && rclcpp::ok()) {
            gui_executor_->spin_some(std::chrono::milliseconds(1));
        }
    });
    gui_ros_spin_timer_->start();

    plot_node_ = std::make_shared<rclcpp::Node>("plot_record_qml_ui_node");
    video_node_ = std::make_shared<rclcpp::Node>("video_qml_ui_node");

    diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 20,
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
        {
            diagnosticsCallback(msg);
        });

    // Video topics and plot topics are discovered automatically from the ROS graph.
    topic_discovery_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(3000),
        [this]()
        {
            discoverVideoTopics();
            discoverPlotTopics();
        });

    executor_manager_ = std::make_unique<asr_sdm_monitor::RosExecutorManager>();
    executor_manager_->addNode(asr_sdm_monitor::RosExecutorRole::Plot, plot_node_);
    executor_manager_->addNode(asr_sdm_monitor::RosExecutorRole::Video, video_node_);

    ros_status_ = "Subscribed: /diagnostics; scanning ROS topics";
    emit rosStatusChanged();

    discoverVideoTopics();
    discoverPlotTopics();
}

void RosUiBridge::addHardwareNode(const rclcpp::Node::SharedPtr & node)
{
    if (!node || !executor_manager_) {
        return;
    }

    executor_manager_->addNode(asr_sdm_monitor::RosExecutorRole::Hardware, node);
}

void RosUiBridge::startRosExecutors()
{
    if (!executor_manager_ || shutting_down_.load()) {
        return;
    }
    executor_manager_->start();
}

void RosUiBridge::addNode(const rclcpp::Node::SharedPtr & node)
{
    addHardwareNode(node);
}

void RosUiBridge::startRosExecutor()
{
    startRosExecutors();
}

RosUiBridge::~RosUiBridge()
{
    shutdown();
}

void RosUiBridge::shutdown()
{
    if (shutting_down_.exchange(true)) {
        return;
    }

    setPlaybackPlaying(false);

    if (plot_update_timer_) {
        plot_update_timer_->stop();
    }
    if (gui_ros_spin_timer_) {
        gui_ros_spin_timer_->stop();
    }
    if (topic_discovery_timer_) {
        topic_discovery_timer_->cancel();
    }
    if (gui_executor_) {
        gui_executor_->cancel();
    }

    // Stop ROS callbacks before releasing subscriptions or the rosbag writer.
    if (executor_manager_) {
        executor_manager_->stop();
    }

    stopPlotSubscriptions();
    stopPlotRecording();
}

QString RosUiBridge::rosStatus() const
{
    return ros_status_;
}
