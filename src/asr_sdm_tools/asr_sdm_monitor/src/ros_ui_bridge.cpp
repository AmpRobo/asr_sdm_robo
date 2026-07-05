#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QStringList>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QUrl>
#include <Qt>
#include <QColor>
#include <QColorDialog>
#include <QFileDialog>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

namespace
{
constexpr int kVideoSlotCount = 4;
constexpr const char *kImageType = "sensor_msgs/msg/Image";
constexpr const char *kCompressedImageType = "sensor_msgs/msg/CompressedImage";
constexpr int kMaxPlotSamples = 600;
constexpr int kPlotUiUpdateIntervalMs = 50;
constexpr int kRecordedPlaybackUpdateIntervalMs = 50;

QString plotFieldPathForTopic(const QString &topicName, const QString &fieldName)
{
    const QString topic = topicName.trimmed();
    const QString field = fieldName.trimmed();
    return field.isEmpty() ? topic : QStringLiteral("%1.%2").arg(topic, field);
}

double fallbackNowMs()
{
    return static_cast<double>(QDateTime::currentMSecsSinceEpoch());
}

double stampToMs(const builtin_interfaces::msg::Time &stamp, double fallbackAbsoluteTimeMs)
{
    const double value = static_cast<double>(stamp.sec) * 1000.0
                         + static_cast<double>(stamp.nanosec) * 1e-6;
    return value > 0.0 ? value : fallbackAbsoluteTimeMs;
}

QVariantMap basePlotSample(double absoluteTimeMs)
{
    QVariantMap sample;
    sample[QStringLiteral("stamp")] = absoluteTimeMs / 1000.0;
    sample[QStringLiteral("absoluteTimeMs")] = absoluteTimeMs;
    return sample;
}

void appendPlotField(QVariantList &fields,
                     const QString &topicName,
                     const QString &topicType,
                     const QString &fieldName,
                     const QString &unit)
{
    QVariantMap field;
    field[QStringLiteral("topic")] = topicName;
    field[QStringLiteral("topicType")] = topicType;
    field[QStringLiteral("field")] = fieldName;
    field[QStringLiteral("path")] = plotFieldPathForTopic(topicName, fieldName);
    field[QStringLiteral("label")] = plotFieldPathForTopic(topicName, fieldName);
    field[QStringLiteral("unit")] = unit;
    fields.append(field);
}

template<typename NumericT>
QVariantMap scalarSample(const QString &topicName, NumericT value, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(fallbackAbsoluteTimeMs);
    sample[plotFieldPathForTopic(topicName, QString())] = static_cast<double>(value);
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Bool &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data ? 1.0 : 0.0, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Float32 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Float64 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Int8 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Int16 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Int32 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::Int64 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::UInt8 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::UInt16 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::UInt32 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const std_msgs::msg::UInt64 &msg, double fallbackAbsoluteTimeMs)
{
    return scalarSample(topicName, msg.data, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::Imu &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular_velocity.x"))] = msg.angular_velocity.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular_velocity.y"))] = msg.angular_velocity.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular_velocity.z"))] = msg.angular_velocity.z;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear_acceleration.x"))] = msg.linear_acceleration.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear_acceleration.y"))] = msg.linear_acceleration.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear_acceleration.z"))] = msg.linear_acceleration.z;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::Temperature &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("temperature"))] = msg.temperature;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("variance"))] = msg.variance;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::FluidPressure &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("fluid_pressure"))] = msg.fluid_pressure;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("variance"))] = msg.variance;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::RelativeHumidity &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("relative_humidity"))] = msg.relative_humidity;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("variance"))] = msg.variance;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::MagneticField &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("magnetic_field.x"))] = msg.magnetic_field.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("magnetic_field.y"))] = msg.magnetic_field.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("magnetic_field.z"))] = msg.magnetic_field.z;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const sensor_msgs::msg::BatteryState &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    sample[plotFieldPathForTopic(topicName, QStringLiteral("voltage"))] = msg.voltage;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("temperature"))] = msg.temperature;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("current"))] = msg.current;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("charge"))] = msg.charge;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("capacity"))] = msg.capacity;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("design_capacity"))] = msg.design_capacity;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("percentage"))] = msg.percentage;
    return sample;
}

QVariantMap vector3Sample(const QString &topicName, const geometry_msgs::msg::Vector3 &vector, double absoluteTimeMs)
{
    QVariantMap sample = basePlotSample(absoluteTimeMs);
    sample[plotFieldPathForTopic(topicName, QStringLiteral("x"))] = vector.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("y"))] = vector.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("z"))] = vector.z;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::Vector3 &msg, double fallbackAbsoluteTimeMs)
{
    return vector3Sample(topicName, msg, fallbackAbsoluteTimeMs);
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::Vector3Stamped &msg, double fallbackAbsoluteTimeMs)
{
    return vector3Sample(topicName, msg.vector, stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::Twist &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(fallbackAbsoluteTimeMs);
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.x"))] = msg.linear.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.y"))] = msg.linear.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.z"))] = msg.linear.z;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.x"))] = msg.angular.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.y"))] = msg.angular.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.z"))] = msg.angular.z;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::TwistStamped &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = sampleFromPlotMessage(topicName, msg.twist, stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::Accel &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = basePlotSample(fallbackAbsoluteTimeMs);
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.x"))] = msg.linear.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.y"))] = msg.linear.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("linear.z"))] = msg.linear.z;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.x"))] = msg.angular.x;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.y"))] = msg.angular.y;
    sample[plotFieldPathForTopic(topicName, QStringLiteral("angular.z"))] = msg.angular.z;
    return sample;
}

QVariantMap sampleFromPlotMessage(const QString &topicName, const geometry_msgs::msg::AccelStamped &msg, double fallbackAbsoluteTimeMs)
{
    QVariantMap sample = sampleFromPlotMessage(topicName, msg.accel, stampToMs(msg.header.stamp, fallbackAbsoluteTimeMs));
    return sample;
}

template<typename MessageT>
bool deserializeRecordedPlotSample(const rosbag2_storage::SerializedBagMessage &bagMessage,
                                   const QString &topicName,
                                   double fallbackAbsoluteTimeMs,
                                   QVariantMap *sample)
{
    if (!sample || !bagMessage.serialized_data) {
        return false;
    }

    MessageT message;
    rclcpp::Serialization<MessageT> serializer;
    rclcpp::SerializedMessage serializedMessage(*bagMessage.serialized_data);
    serializer.deserialize_message(&serializedMessage, &message);
    *sample = sampleFromPlotMessage(topicName, message, fallbackAbsoluteTimeMs);
    return true;
}
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

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    ros_status_ = "Subscribed: /diagnostics; scanning ROS topics";
    emit rosStatusChanged();

    discoverVideoTopics();
    discoverPlotTopics();
}

void RosUiBridge::addNode(const rclcpp::Node::SharedPtr & node)
{
    if (!node || !executor_) {
        return;
    }
    owned_nodes_.push_back(node);
    executor_->add_node(node);
}

void RosUiBridge::startRosExecutor()
{
    if (ros_executor_started_ || !executor_) {
        return;
    }
    ros_executor_started_ = true;
    ros_thread_ = std::thread([this]()
    {
        executor_->spin();
    });
}

RosUiBridge::~RosUiBridge()
{
    stopPlotRecording();
    setPlaybackPlaying(false);
    stopPlotSubscriptions();

    if (executor_) {
        executor_->cancel();
    }

    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
}

QString RosUiBridge::rosStatus() const
{
    return ros_status_;
}

QVariantMap RosUiBridge::cpuSummary() const
{
    return cpu_summary_;
}

QVariantList RosUiBridge::cpuHistory() const
{
    return cpu_history_;
}

QVariantList RosUiBridge::cpuCoreRows() const
{
    return cpu_core_rows_;
}

QVariantMap RosUiBridge::memorySummary() const
{
    return memory_summary_;
}

QVariantList RosUiBridge::memoryHistory() const
{
    return memory_history_;
}

QVariantList RosUiBridge::memoryRows() const
{
    return memory_rows_;
}

QVariantMap RosUiBridge::hddSummary() const
{
    return hdd_summary_;
}

QVariantList RosUiBridge::hddRows() const
{
    return hdd_rows_;
}

QVariantMap RosUiBridge::netSummary() const
{
    return net_summary_;
}

QVariantList RosUiBridge::netInHistory() const
{
    return net_in_history_;
}

QVariantList RosUiBridge::netOutHistory() const
{
    return net_out_history_;
}

QVariantList RosUiBridge::netInterfaceRows() const
{
    return net_interface_rows_;
}

QVariantMap RosUiBridge::ntpSummary() const
{
    return ntp_summary_;
}

QVariantList RosUiBridge::ntpRows() const
{
    return ntp_rows_;
}

QVariantList RosUiBridge::videoTopics() const
{
    return video_topics_;
}

QVariantList RosUiBridge::videoSlots() const
{
    QVariantList video_slot_list;
    for (int slot_index = 0; slot_index < kVideoSlotCount; ++slot_index) {
        const auto &slot = video_slots_[static_cast<size_t>(slot_index)];
        QVariantMap item;
        item[QStringLiteral("topic")] = slot.topic;
        item[QStringLiteral("topicType")] = slot.topic_type;
        item[QStringLiteral("status")] = slot.status;
        item[QStringLiteral("frameRevision")] = slot.frame_revision;
        video_slot_list.append(item);
    }
    return video_slot_list;
}

QString RosUiBridge::videoTopic0() const
{
    return video_slots_[0].topic;
}

QString RosUiBridge::videoTopic1() const
{
    return video_slots_[1].topic;
}

QString RosUiBridge::videoTopic2() const
{
    return video_slots_[2].topic;
}

QString RosUiBridge::videoTopic3() const
{
    return video_slots_[3].topic;
}

QString RosUiBridge::videoStatus0() const
{
    return video_slots_[0].status;
}

QString RosUiBridge::videoStatus1() const
{
    return video_slots_[1].status;
}

QString RosUiBridge::videoStatus2() const
{
    return video_slots_[2].status;
}

QString RosUiBridge::videoStatus3() const
{
    return video_slots_[3].status;
}

int RosUiBridge::videoFrame0Revision() const
{
    return video_slots_[0].frame_revision;
}

int RosUiBridge::videoFrame1Revision() const
{
    return video_slots_[1].frame_revision;
}

int RosUiBridge::videoFrame2Revision() const
{
    return video_slots_[2].frame_revision;
}

int RosUiBridge::videoFrame3Revision() const
{
    return video_slots_[3].frame_revision;
}

QImage RosUiBridge::videoFrameImage(int slotIndex) const
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return {};
    }

    std::lock_guard<std::mutex> lock(video_frame_mutex_);
    return video_slots_[static_cast<size_t>(slotIndex)].frame.copy();
}

QVariantList RosUiBridge::plotTopics() const
{
    return plot_topics_;
}

QString RosUiBridge::plotTopicsStatus() const
{
    return plot_topics_status_;
}

QVariantList RosUiBridge::plotFieldOptions() const
{
    return plot_field_options_;
}

QVariantList RosUiBridge::imuPlotSamples() const
{
    return imu_plot_samples_;
}

QString RosUiBridge::plotStatus() const
{
    return plot_status_;
}

QString RosUiBridge::plotDataSource() const
{
    return plot_data_source_;
}

QVariantList RosUiBridge::recordedPlotFieldOptions() const
{
    return recorded_plot_field_options_;
}

QVariantList RosUiBridge::recordedPlotSamples() const
{
    return recorded_plot_samples_;
}

bool RosUiBridge::plotRecording() const
{
    return plot_recording_;
}

QString RosUiBridge::plotRecordingPath() const
{
    return plot_recording_path_;
}

QString RosUiBridge::recordedFilePath() const
{
    return recorded_file_path_;
}

QString RosUiBridge::recordedStatus() const
{
    return recorded_status_;
}

double RosUiBridge::playbackStartTimeMs() const
{
    return playback_start_time_ms_;
}

double RosUiBridge::playbackEndTimeMs() const
{
    return playback_end_time_ms_;
}

double RosUiBridge::playbackCurrentTimeMs() const
{
    return playback_current_time_ms_;
}

double RosUiBridge::playbackSpeed() const
{
    return playback_speed_;
}

bool RosUiBridge::playbackPlaying() const
{
    return playback_playing_;
}

void RosUiBridge::setPlotDataSource(const QString &dataSource)
{
    const QString normalized = dataSource.trimmed().toLower();
    const QString next = normalized == QStringLiteral("recorded") ? QStringLiteral("recorded") : QStringLiteral("live");
    if (plot_data_source_ == next) {
        return;
    }

    plot_data_source_ = next;
    emit plotDataSourceChanged();
}

QString RosUiBridge::defaultPlotRecordingPath() const
{
    const QString directory = QDir::homePath() + QStringLiteral("/asr_sdm_monitor_recordings");
    QDir().mkpath(directory);
    const QString stamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return directory + QStringLiteral("/plot_") + stamp;
}

QString RosUiBridge::pickExistingDirectory(const QString &title, const QString &startDir)
{
    QString initialDir = normalizeLocalPath(startDir);
    if (initialDir.isEmpty()) {
        initialDir = QDir::homePath();
    } else {
        const QFileInfo info(initialDir);
        initialDir = info.isDir() ? info.absoluteFilePath() : info.absolutePath();
    }

    return QFileDialog::getExistingDirectory(nullptr, title, initialDir);
}

QString RosUiBridge::pickColor(const QString &title, const QString &initialColor)
{
    const QColor initial(initialColor);
    const QColor selected = QColorDialog::getColor(
        initial.isValid() ? initial : QColor(QStringLiteral("#3794ff")),
        nullptr,
        title);
    return selected.isValid() ? selected.name(QColor::HexRgb) : QString();
}

QString RosUiBridge::normalizeLocalPath(const QString &filePath)
{
    QString path = filePath.trimmed();
    if (path.startsWith(QStringLiteral("file:"))) {
        const QUrl url(path);
        if (url.isLocalFile()) {
            path = url.toLocalFile();
        }
    }

    if (path.startsWith(QStringLiteral("~/"))) {
        path = QDir::homePath() + path.mid(1);
    }

    if (path.isEmpty()) {
        return {};
    }

    return QDir::cleanPath(path);
}

bool RosUiBridge::startPlotRecording(const QString &filePath)
{
    QString path = normalizeLocalPath(filePath);
    if (path.isEmpty()) {
        path = defaultPlotRecordingPath();
    }

    stopPlotRecording();

    const QFileInfo info(path);
    const QString parent_dir = info.absolutePath();
    if (!QDir().mkpath(parent_dir)) {
        plot_status_ = QStringLiteral("Cannot create recording directory: %1").arg(parent_dir);
        emit plotStatusChanged();
        return false;
    }

    if (QFileInfo::exists(path)) {
        plot_status_ = QStringLiteral("Bag path already exists: %1").arg(path);
        emit plotStatusChanged();
        return false;
    }

    try {
        auto writer = std::make_unique<rosbag2_cpp::Writer>();
        writer->open(path.toStdString());

        {
            std::lock_guard<std::mutex> lock(plot_recording_mutex_);
            plot_bag_writer_ = std::move(writer);
            plot_recorded_message_count_ = 0;
            plot_recording_ = true;
            plot_recording_path_ = path;
        }

        plot_status_ = QStringLiteral("Recording bag: %1").arg(path);
        emit plotRecordingChanged();
        emit plotStatusChanged();
        return true;
    } catch (const std::exception &error) {
        plot_status_ = QStringLiteral("Cannot start bag recording: %1").arg(QString::fromUtf8(error.what()));
        emit plotStatusChanged();
        return false;
    }
}

void RosUiBridge::stopPlotRecording()
{
    bool was_recording = false;
    QString saved_path;
    size_t saved_count = 0;

    {
        std::lock_guard<std::mutex> lock(plot_recording_mutex_);
        was_recording = plot_recording_;
        saved_path = plot_recording_path_;
        saved_count = plot_recorded_message_count_;
        plot_recording_ = false;
        plot_bag_writer_.reset();
    }

    if (!was_recording) {
        return;
    }

    plot_status_ = QStringLiteral("Bag saved: %1 (%2 messages)").arg(saved_path).arg(saved_count);
    emit plotRecordingChanged();
    emit plotStatusChanged();
}

template<typename MessageT>
void RosUiBridge::writePlotRecordingSample(const QString &topicName, const MessageT &msg, double absoluteTimeMs)
{
    std::lock_guard<std::mutex> lock(plot_recording_mutex_);
    if (!plot_recording_ || !plot_bag_writer_) {
        return;
    }

    try {
        int64_t timestamp_ns = static_cast<int64_t>(std::llround(absoluteTimeMs * 1000000.0));
        if (timestamp_ns <= 0 && node_) {
            timestamp_ns = node_->now().nanoseconds();
        }

        plot_bag_writer_->write(msg, topicName.toStdString(), rclcpp::Time(timestamp_ns));
        ++plot_recorded_message_count_;
    } catch (const std::exception &error) {
        plot_recording_ = false;
        plot_status_ = QStringLiteral("Bag recording stopped: %1").arg(QString::fromUtf8(error.what()));
        plot_bag_writer_.reset();
        QMetaObject::invokeMethod(
            this,
            [this]()
            {
                emit plotRecordingChanged();
                emit plotStatusChanged();
            },
            Qt::QueuedConnection);
    }
}

bool RosUiBridge::openRecordedPlotFile(const QString &filePath)
{
    const QString path = normalizeLocalPath(filePath);
    if (path.isEmpty()) {
        recorded_status_ = QStringLiteral("Recorded file path is empty");
        emit recordedPlaybackChanged();
        return false;
    }

    setPlaybackPlaying(false);

    const bool ok = loadRecordedRosbag(path);

    if (ok) {
        setPlotDataSource(QStringLiteral("recorded"));
    }

    emit recordedPlaybackChanged();
    emit recordedPlotFieldOptionsChanged();
    emit recordedPlotSamplesChanged();
    emit playbackCurrentTimeMsChanged();
    return ok;
}

bool RosUiBridge::loadRecordedRosbag(const QString &filePath)
{
    try {
        rosbag2_cpp::Reader reader;
        reader.open(filePath.toStdString());

        QMap<QString, QString> bagTopicTypes;
        QVariantList availableFields;
        const auto topicsAndTypes = reader.get_all_topics_and_types();
        for (const auto &topicMetadata : topicsAndTypes) {
            const QString topicName = QString::fromStdString(topicMetadata.name).trimmed();
            const QString topicType = QString::fromStdString(topicMetadata.type).trimmed();
            if (topicName.isEmpty() || topicType.isEmpty()) {
                continue;
            }

            bagTopicTypes.insert(topicName, topicType);
            const QVariantList fields = plotFieldOptionsForTopic(topicName, topicType);
            for (const QVariant &field : fields) {
                availableFields.append(field);
            }
        }

        QVariantList samples;
        double bagStartMs = -1.0;
        double bagEndMs = -1.0;

        while (reader.has_next()) {
            const auto bagMessage = reader.read_next();
            if (!bagMessage) {
                continue;
            }

            const QString topicName = QString::fromStdString(bagMessage->topic_name).trimmed();
            const QString topicType = bagTopicTypes.value(topicName);
            const double messageTimeMs = static_cast<double>(bagMessage->recv_timestamp) / 1000000.0;
            if (messageTimeMs >= 0.0) {
                if (bagStartMs < 0.0 || messageTimeMs < bagStartMs) {
                    bagStartMs = messageTimeMs;
                }
                if (bagEndMs < 0.0 || messageTimeMs > bagEndMs) {
                    bagEndMs = messageTimeMs;
                }
            }

            if (!isSupportedPlotType(topicType)) {
                continue;
            }

            QVariantMap sample;
            bool decoded = false;
#define DECODE_PLOT_SAMPLE(TYPE_STRING, MESSAGE_TYPE) \
            if (!decoded && topicType == QStringLiteral(TYPE_STRING)) { \
                decoded = deserializeRecordedPlotSample<MESSAGE_TYPE>(*bagMessage, topicName, messageTimeMs, &sample); \
            }
            DECODE_PLOT_SAMPLE("std_msgs/msg/Bool", std_msgs::msg::Bool)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Float32", std_msgs::msg::Float32)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Float64", std_msgs::msg::Float64)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Int8", std_msgs::msg::Int8)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Int16", std_msgs::msg::Int16)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Int32", std_msgs::msg::Int32)
            DECODE_PLOT_SAMPLE("std_msgs/msg/Int64", std_msgs::msg::Int64)
            DECODE_PLOT_SAMPLE("std_msgs/msg/UInt8", std_msgs::msg::UInt8)
            DECODE_PLOT_SAMPLE("std_msgs/msg/UInt16", std_msgs::msg::UInt16)
            DECODE_PLOT_SAMPLE("std_msgs/msg/UInt32", std_msgs::msg::UInt32)
            DECODE_PLOT_SAMPLE("std_msgs/msg/UInt64", std_msgs::msg::UInt64)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/Imu", sensor_msgs::msg::Imu)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/Temperature", sensor_msgs::msg::Temperature)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/FluidPressure", sensor_msgs::msg::FluidPressure)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/RelativeHumidity", sensor_msgs::msg::RelativeHumidity)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/MagneticField", sensor_msgs::msg::MagneticField)
            DECODE_PLOT_SAMPLE("sensor_msgs/msg/BatteryState", sensor_msgs::msg::BatteryState)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/Vector3", geometry_msgs::msg::Vector3)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/Vector3Stamped", geometry_msgs::msg::Vector3Stamped)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/Twist", geometry_msgs::msg::Twist)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/TwistStamped", geometry_msgs::msg::TwistStamped)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/Accel", geometry_msgs::msg::Accel)
            DECODE_PLOT_SAMPLE("geometry_msgs/msg/AccelStamped", geometry_msgs::msg::AccelStamped)
#undef DECODE_PLOT_SAMPLE

            if (decoded && !sample.isEmpty()) {
                samples.append(sample);
            }
        }

        if (bagStartMs < 0.0 || bagEndMs < 0.0) {
            recorded_status_ = QStringLiteral("No messages found in bag: %1").arg(filePath);
            recorded_available_plot_field_options_.clear();
            recorded_plot_field_options_.clear();
            return false;
        }

        std::sort(samples.begin(), samples.end(), [](const QVariant &a, const QVariant &b)
        {
            return a.toMap().value(QStringLiteral("absoluteTimeMs")).toDouble()
                   < b.toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
        });

        for (QVariant &item : samples) {
            QVariantMap sample = item.toMap();
            sample[QStringLiteral("relativeTime")] = (sample.value(QStringLiteral("absoluteTimeMs")).toDouble() - bagStartMs) / 1000.0;
            item = sample;
        }

        recorded_file_path_ = filePath;
        recorded_plot_samples_ = samples;
        recorded_available_plot_field_options_ = availableFields;
        recorded_plot_field_options_ = filterPlotFieldOptionsForSelectedTopics(recorded_available_plot_field_options_);
        recorded_bag_start_time_ms_ = bagStartMs;
        recorded_bag_end_time_ms_ = bagEndMs;
        playback_start_time_ms_ = bagStartMs;
        playback_end_time_ms_ = bagEndMs;
        playback_current_time_ms_ = playback_start_time_ms_;
        recorded_status_ = samples.isEmpty()
                               ? QStringLiteral("Loaded bag: %1; no supported plot topics found").arg(filePath)
                               : QStringLiteral("Loaded bag: %1 (%2 plot samples)").arg(filePath).arg(samples.size());
        return true;
    } catch (const std::exception &error) {
        recorded_status_ = QStringLiteral("Cannot load rosbag / MCAP: %1").arg(QString::fromUtf8(error.what()));
        return false;
    }
}

void RosUiBridge::updateRecordedPlaybackBounds()
{
    setPlaybackPlaying(false);

    if (recorded_plot_samples_.isEmpty()) {
        recorded_bag_start_time_ms_ = 0.0;
        recorded_bag_end_time_ms_ = 0.0;
        playback_start_time_ms_ = 0.0;
        playback_end_time_ms_ = 0.0;
        playback_current_time_ms_ = 0.0;
        return;
    }

    recorded_bag_start_time_ms_ = recorded_plot_samples_.first().toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
    recorded_bag_end_time_ms_ = recorded_plot_samples_.last().toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
    playback_start_time_ms_ = recorded_bag_start_time_ms_;
    playback_end_time_ms_ = recorded_bag_end_time_ms_;
    playback_current_time_ms_ = playback_start_time_ms_;
}

void RosUiBridge::setPlaybackStartTimeMs(double startTimeMs)
{
    if (recorded_bag_end_time_ms_ <= recorded_bag_start_time_ms_) {
        return;
    }

    const double upper = std::min(playback_end_time_ms_, recorded_bag_end_time_ms_);
    const double clamped = std::clamp(startTimeMs, recorded_bag_start_time_ms_, upper);
    if (std::abs(playback_start_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_start_time_ms_ = clamped;
    bool currentChanged = false;
    if (playback_current_time_ms_ < playback_start_time_ms_) {
        playback_current_time_ms_ = playback_start_time_ms_;
        currentChanged = true;
    }

    emit recordedPlaybackChanged();
    if (currentChanged) {
        emit playbackCurrentTimeMsChanged();
    }
}

void RosUiBridge::setPlaybackEndTimeMs(double endTimeMs)
{
    if (recorded_bag_end_time_ms_ <= recorded_bag_start_time_ms_) {
        return;
    }

    const double lower = std::max(playback_start_time_ms_, recorded_bag_start_time_ms_);
    const double clamped = std::clamp(endTimeMs, lower, recorded_bag_end_time_ms_);
    if (std::abs(playback_end_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_end_time_ms_ = clamped;
    bool currentChanged = false;
    if (playback_current_time_ms_ > playback_end_time_ms_) {
        playback_current_time_ms_ = playback_end_time_ms_;
        currentChanged = true;
    }

    emit recordedPlaybackChanged();
    if (currentChanged) {
        emit playbackCurrentTimeMsChanged();
    }
}

void RosUiBridge::setPlaybackCurrentTimeMs(double currentTimeMs)
{
    const double clamped = std::clamp(currentTimeMs, playback_start_time_ms_, playback_end_time_ms_);
    if (std::abs(playback_current_time_ms_ - clamped) < 0.5) {
        return;
    }

    playback_current_time_ms_ = clamped;
    emit playbackCurrentTimeMsChanged();
}

void RosUiBridge::setPlaybackSpeed(double speed)
{
    const double next = std::clamp(speed, 0.05, 20.0);
    if (std::abs(playback_speed_ - next) < 1e-6) {
        return;
    }

    playback_speed_ = next;
    emit playbackSpeedChanged();
}

void RosUiBridge::setPlaybackPlaying(bool playing)
{
    if (playing && playback_end_time_ms_ <= playback_start_time_ms_) {
        playing = false;
    }

    if (playing && playback_current_time_ms_ >= playback_end_time_ms_ - 0.5) {
        playback_current_time_ms_ = playback_start_time_ms_;
        emit playbackCurrentTimeMsChanged();
    }

    if (playback_playing_ == playing) {
        return;
    }

    playback_playing_ = playing;
    if (playback_timer_) {
        if (playback_playing_) {
            playback_last_tick_ = std::chrono::steady_clock::now();
            playback_timer_->start();
        } else {
            playback_timer_->stop();
        }
    }

    emit playbackPlayingChanged();
}

void RosUiBridge::playbackTick()
{
    if (!playback_playing_) {
        return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double elapsedMs = std::chrono::duration<double, std::milli>(now - playback_last_tick_).count();
    playback_last_tick_ = now;

    double next = playback_current_time_ms_ + elapsedMs * playback_speed_;
    if (next >= playback_end_time_ms_) {
        next = playback_end_time_ms_;
        playback_current_time_ms_ = next;
        emit playbackCurrentTimeMsChanged();
        setPlaybackPlaying(false);
        return;
    }

    playback_current_time_ms_ = next;
    emit playbackCurrentTimeMsChanged();
}

void RosUiBridge::refreshPlotTopics()
{
    discoverPlotTopics();
}

void RosUiBridge::setPlotTopicSelected(const QString &topicName, bool selected)
{
    const QString normalized = topicName.trimmed();
    if (normalized.isEmpty()) {
        return;
    }

    const bool alreadySelected = selected_plot_topic_names_.contains(normalized);
    if (selected == alreadySelected) {
        return;
    }

    if (selected) {
        selected_plot_topic_names_.append(normalized);
        selected_plot_topic_names_.removeDuplicates();
        selected_plot_topic_names_.sort(Qt::CaseSensitive);
    } else {
        selected_plot_topic_names_.removeAll(normalized);
    }

    {
        std::lock_guard<std::mutex> lock(live_plot_mutex_);
        pending_live_plot_samples_.clear();
        pending_live_plot_status_topic_.clear();
    }
    imu_plot_samples_.clear();
    plot_start_time_ = -1.0;

    rebuildPlotTopicsModel();
    rebuildPlotFieldOptions();
    refreshPlotSubscriptions();
    emit imuPlotSamplesChanged();
}

void RosUiBridge::discoverPlotTopics()
{
    if (!node_) {
        return;
    }

    QStringList discovered_names;
    QMap<QString, QString> discovered_types;

    const auto topics_and_types = node_->get_topic_names_and_types();
    for (const auto &entry : topics_and_types) {
        const QString topic_name = QString::fromStdString(entry.first).trimmed();
        if (topic_name.isEmpty()) {
            continue;
        }

        discovered_names.append(topic_name);
        if (!entry.second.empty()) {
            discovered_types.insert(topic_name, QString::fromStdString(entry.second.front()).trimmed());
        }
    }

    QMetaObject::invokeMethod(
        this,
        [this, discovered_names, discovered_types]()
        {
            applyDiscoveredPlotTopics(discovered_names, discovered_types);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::applyDiscoveredPlotTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes)
{
    QStringList discovered_names = topicNames;
    discovered_names.removeDuplicates();
    discovered_names.sort(Qt::CaseSensitive);

    QMap<QString, QString> discovered_types;
    for (const QString &name : discovered_names) {
        discovered_types.insert(name, topicTypes.value(name));
    }

    const bool topicGraphChanged = plot_topic_names_ != discovered_names || plot_topic_types_ != discovered_types;
    plot_topic_names_ = discovered_names;
    plot_topic_types_ = discovered_types;

    rebuildPlotTopicsModel();
    if (topicGraphChanged) {
        rebuildPlotFieldOptions();
        refreshPlotSubscriptions();
    }
}

void RosUiBridge::rebuildPlotTopicsModel()
{
    QVariantList topic_items;
    int selectedCount = 0;
    int plottableFieldCount = 0;

    for (const QString &name : plot_topic_names_) {
        const QString type = plot_topic_types_.value(name);
        const QVariantList fields = plotFieldOptionsForTopic(name, type);
        const bool selected = selected_plot_topic_names_.contains(name);
        if (selected) {
            ++selectedCount;
            plottableFieldCount += fields.size();
        }

        QVariantMap item;
        item[QStringLiteral("name")] = name;
        item[QStringLiteral("type")] = type;
        item[QStringLiteral("selected")] = selected;
        item[QStringLiteral("plottable")] = !fields.isEmpty();
        item[QStringLiteral("fieldCount")] = fields.size();
        topic_items.append(item);
    }

    plot_topics_ = topic_items;
    plot_topics_status_ = QStringLiteral("%1 topics, %2 selected, %3 plottable fields")
                              .arg(plot_topic_names_.size())
                              .arg(selectedCount)
                              .arg(plottableFieldCount);
    emit plotTopicsChanged();
}

QVariantList RosUiBridge::filterPlotFieldOptionsForSelectedTopics(const QVariantList &fields) const
{
    QVariantList result;
    for (const QVariant &candidate : fields) {
        const QVariantMap field = candidate.toMap();
        const QString topic = field.value(QStringLiteral("topic")).toString();
        if (!topic.isEmpty() && selected_plot_topic_names_.contains(topic)) {
            result.append(field);
        }
    }
    return result;
}

void RosUiBridge::rebuildPlotFieldOptions()
{
    QVariantList fields;
    for (const QString &topic : selected_plot_topic_names_) {
        const QString type = plot_topic_types_.value(topic);
        const QVariantList topicFields = plotFieldOptionsForTopic(topic, type);
        for (const QVariant &field : topicFields) {
            fields.append(field);
        }
    }

    plot_field_options_ = fields;
    recorded_plot_field_options_ = filterPlotFieldOptionsForSelectedTopics(recorded_available_plot_field_options_);
    emit plotFieldOptionsChanged();
    emit recordedPlotFieldOptionsChanged();
}

void RosUiBridge::stopPlotSubscriptions()
{
    plot_subscriptions_.clear();
}

void RosUiBridge::refreshPlotSubscriptions()
{
    stopPlotSubscriptions();

    if (!node_) {
        return;
    }

    for (const QString &topic : selected_plot_topic_names_) {
        const QString type = plot_topic_types_.value(topic);
        if (isSupportedPlotType(type)) {
            startPlotSubscriptionForTopic(topic, type);
        }
    }
}

template<typename MessageT>
void RosUiBridge::startTypedPlotSubscription(const QString &topicName)
{
    const std::string topic = topicName.toStdString();
    auto subscription = node_->create_subscription<MessageT>(
        topic, rclcpp::SensorDataQoS(),
        [this, topicName](typename MessageT::SharedPtr msg)
        {
            if (!msg) {
                return;
            }

            const double fallbackAbsoluteTimeMs = node_
                                                   ? static_cast<double>(node_->now().nanoseconds()) / 1000000.0
                                                   : fallbackNowMs();
            const QVariantMap sample = sampleFromPlotMessage(topicName, *msg, fallbackAbsoluteTimeMs);
            bool absoluteTimeOk = false;
            const double absoluteTimeValueMs = sample.value(QStringLiteral("absoluteTimeMs")).toDouble(&absoluteTimeOk);
            const double absoluteTimeMs = absoluteTimeOk ? absoluteTimeValueMs : fallbackAbsoluteTimeMs;
            writePlotRecordingSample(topicName, *msg, absoluteTimeMs);
            appendLivePlotSample(sample, topicName);
        });

    plot_subscriptions_.insert(topicName, std::static_pointer_cast<rclcpp::SubscriptionBase>(subscription));
}

void RosUiBridge::startPlotSubscriptionForTopic(const QString &topicName, const QString &topicType)
{
#define START_PLOT_SUBSCRIPTION(TYPE_STRING, MESSAGE_TYPE) \
    if (topicType == QStringLiteral(TYPE_STRING)) { \
        startTypedPlotSubscription<MESSAGE_TYPE>(topicName); \
        return; \
    }
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Bool", std_msgs::msg::Bool)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Float32", std_msgs::msg::Float32)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Float64", std_msgs::msg::Float64)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Int8", std_msgs::msg::Int8)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Int16", std_msgs::msg::Int16)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Int32", std_msgs::msg::Int32)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/Int64", std_msgs::msg::Int64)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/UInt8", std_msgs::msg::UInt8)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/UInt16", std_msgs::msg::UInt16)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/UInt32", std_msgs::msg::UInt32)
    START_PLOT_SUBSCRIPTION("std_msgs/msg/UInt64", std_msgs::msg::UInt64)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/Imu", sensor_msgs::msg::Imu)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/Temperature", sensor_msgs::msg::Temperature)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/FluidPressure", sensor_msgs::msg::FluidPressure)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/RelativeHumidity", sensor_msgs::msg::RelativeHumidity)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/MagneticField", sensor_msgs::msg::MagneticField)
    START_PLOT_SUBSCRIPTION("sensor_msgs/msg/BatteryState", sensor_msgs::msg::BatteryState)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/Vector3", geometry_msgs::msg::Vector3)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/Vector3Stamped", geometry_msgs::msg::Vector3Stamped)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/Twist", geometry_msgs::msg::Twist)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/TwistStamped", geometry_msgs::msg::TwistStamped)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/Accel", geometry_msgs::msg::Accel)
    START_PLOT_SUBSCRIPTION("geometry_msgs/msg/AccelStamped", geometry_msgs::msg::AccelStamped)
#undef START_PLOT_SUBSCRIPTION
}

void RosUiBridge::appendLivePlotSample(const QVariantMap &sample, const QString &topicName)
{
    if (sample.isEmpty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(live_plot_mutex_);
    pending_live_plot_samples_.append(sample);
    pending_live_plot_status_topic_ = topicName;
    while (pending_live_plot_samples_.size() > kMaxPlotSamples) {
        pending_live_plot_samples_.removeFirst();
    }
}

void RosUiBridge::flushLivePlotSamples()
{
    QVariantList pending_samples;
    QString latest_topic;
    {
        std::lock_guard<std::mutex> lock(live_plot_mutex_);
        if (pending_live_plot_samples_.isEmpty()) {
            return;
        }
        pending_samples.swap(pending_live_plot_samples_);
        latest_topic = pending_live_plot_status_topic_;
    }

    for (const QVariant &item : pending_samples) {
        QVariantMap stored_sample = item.toMap();
        if (stored_sample.isEmpty()) {
            continue;
        }

        const double stamp = stored_sample.value(QStringLiteral("stamp")).toDouble();
        if (plot_start_time_ < 0.0) {
            plot_start_time_ = stamp;
        }

        stored_sample[QStringLiteral("relativeTime")] = stamp - plot_start_time_;
        imu_plot_samples_.append(stored_sample);
    }

    while (imu_plot_samples_.size() > kMaxPlotSamples) {
        imu_plot_samples_.removeFirst();
    }

    bool recording_now = false;
    QString recording_path;
    size_t recorded_count = 0;
    {
        std::lock_guard<std::mutex> lock(plot_recording_mutex_);
        recording_now = plot_recording_;
        recording_path = plot_recording_path_;
        recorded_count = plot_recorded_message_count_;
    }

    plot_status_ = recording_now
                       ? QStringLiteral("Recording bag: %1 (%2 messages)").arg(recording_path).arg(recorded_count)
                       : QStringLiteral("Receiving: %1 (%2 samples)").arg(latest_topic).arg(imu_plot_samples_.size());
    emit imuPlotSamplesChanged();
    emit plotStatusChanged();
}

void RosUiBridge::discoverVideoTopics()
{
    if (!node_) {
        return;
    }

    QStringList discovered_names;
    QMap<QString, QString> discovered_types;

    const auto topics_and_types = node_->get_topic_names_and_types();
    for (const auto &entry : topics_and_types) {
        const QString topic_name = QString::fromStdString(entry.first).trimmed();
        for (const std::string &type_string : entry.second) {
            const QString topic_type = QString::fromStdString(type_string).trimmed();
            if (isPerceptionImageTopic(topic_name, topic_type)) {
                discovered_names.append(topic_name);
                discovered_types.insert(topic_name, topic_type);
                break;
            }
        }
    }

    QMetaObject::invokeMethod(
        this,
        [this, discovered_names, discovered_types]()
        {
            applyDiscoveredVideoTopics(discovered_names, discovered_types);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::setVideoTopic(int slotIndex, const QString &topicName)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QString normalized_topic = topicName.trimmed();

    if (!normalized_topic.isEmpty() && !video_topic_names_.contains(normalized_topic)) {
        return;
    }

    for (int other_slot_index = 0; other_slot_index < kVideoSlotCount; ++other_slot_index) {
        if (other_slot_index == slotIndex) {
            continue;
        }

        auto &other_slot = video_slots_[static_cast<size_t>(other_slot_index)];
        if (!normalized_topic.isEmpty() && other_slot.topic == normalized_topic) {
            stopVideoSlot(other_slot_index);
            other_slot.topic.clear();
            other_slot.topic_type.clear();
            other_slot.status = QStringLiteral("No topic selected");
            emitVideoSlotChanged(other_slot_index);
        }
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    if (slot.topic == normalized_topic) {
        return;
    }

    stopVideoSlot(slotIndex);
    slot.topic = normalized_topic;
    slot.topic_type = video_topic_types_.value(normalized_topic);
    slot.status = normalized_topic.isEmpty() ? QStringLiteral("No topic selected")
                                           : QStringLiteral("Waiting for video frame");

    emitVideoSlotChanged(slotIndex);

    if (!normalized_topic.isEmpty()) {
        startVideoSlot(slotIndex);
    }
}

void RosUiBridge::diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    QMetaObject::invokeMethod(
        this,
        [this]()
        {
            ros_status_ = "Receiving: /diagnostics";
            emit rosStatusChanged();
        },
        Qt::QueuedConnection);

    for (const auto &status : msg->status) {
        const QString name = QString::fromStdString(status.name).toLower();

        if (name.contains("cpu usage")) {
            updateCpu(status);
        } else if (name.contains("memory usage")) {
            updateMemory(status);
        } else if (name.contains("hdd usage")) {
            updateHdd(status);
        } else if (name.contains("network usage")) {
            updateNet(status);
        } else if (name.contains("ntp offset")) {
            updateNtp(status);
        }
    }
}

QString RosUiBridge::valueForKey(const diagnostic_msgs::msg::DiagnosticStatus &status, const QString &key)
{
    for (const auto &item : status.values) {
        if (QString::fromStdString(item.key) == key) {
            return QString::fromStdString(item.value);
        }
    }
    return {};
}

double RosUiBridge::extractNumber(const QString &text)
{
    static const std::regex pattern(R"((-?\d+(?:\.\d+)?))");
    std::smatch match;
    const std::string input = text.toStdString();
    if (std::regex_search(input, match, pattern)) {
        return std::stod(match.str(1));
    }
    return 0.0;
}

QString RosUiBridge::formatNumber(double value, int precision)
{
    return QString::number(value, 'f', precision);
}

QString RosUiBridge::levelToText(unsigned char level)
{
    switch (level) {
    case 0:
        return "OK";
    case 1:
        return "WARN";
    case 2:
        return "ERROR";
    case 3:
        return "STALE";
    default:
        return "UNKNOWN";
    }
}

void RosUiBridge::appendHistory(QVariantList &history, double value, int maxPoints)
{
    history.append(value);
    while (history.size() > maxPoints) {
        history.removeFirst();
    }
}

void RosUiBridge::updateCpu(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    struct CoreData {
        int index = -1;
        QString status;
        QString clock;
        QString user;
        QString system;
        QString idle;
        double usage = 0.0;
        double clockValue = 0.0;
    };

    std::map<int, CoreData> cores;
    static const std::regex core_pattern(R"(Core\s+(\d+)\s+(.+))");

    for (const auto &item : status.values) {
        std::smatch match;
        const std::string key = item.key;
        if (!std::regex_match(key, match, core_pattern)) {
            continue;
        }

        const int idx = std::stoi(match.str(1));
        const QString field = QString::fromStdString(match.str(2));
        auto &core = cores[idx];
        core.index = idx;

        const QString value = QString::fromStdString(item.value);
        if (field == "Status") {
            core.status = value;
        } else if (field == "Clock Speed") {
            core.clock = value;
            core.clockValue = extractNumber(value);
        } else if (field == "User") {
            core.user = value;
        } else if (field == "System") {
            core.system = value;
        } else if (field == "Idle") {
            core.idle = value;
            core.usage = std::clamp(100.0 - extractNumber(value), 0.0, 100.0);
        }
    }

    QVariantList rows;
    std::vector<double> usages;
    std::vector<double> clocks;
    double max_usage = 0.0;

    for (const auto &[index, core] : cores) {
        Q_UNUSED(index);
        QVariantMap row;
        row["core"] = QString::number(core.index);
        row["usage"] = formatNumber(core.usage, 1) + "%";
        row["clock"] = core.clock;
        row["user"] = core.user;
        row["system"] = core.system;
        row["idle"] = core.idle;
        row["status"] = core.status;
        rows.append(row);

        usages.push_back(core.usage);
        clocks.push_back(core.clockValue);
        max_usage = std::max(max_usage, core.usage);
    }

    const double avg_usage = usages.empty()
                                 ? 0.0
                                 : std::accumulate(usages.begin(), usages.end(), 0.0) / usages.size();
    const double avg_clock = clocks.empty()
                                 ? 0.0
                                 : std::accumulate(clocks.begin(), clocks.end(), 0.0) / clocks.size();

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["avgUsage"] = formatNumber(avg_usage, 1) + "%";
    summary["maxUsage"] = formatNumber(max_usage, 1) + "%";
    summary["avgClock"] = formatNumber(avg_clock, 0) + " MHz";
    summary["load1"] = valueForKey(status, "Load Average (1min)");
    summary["load5"] = valueForKey(status, "Load Average (5min)");
    summary["load15"] = valueForKey(status, "Load Average (15min)");
    summary["coreCount"] = static_cast<int>(rows.size());

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, avg_usage]()
        {
            cpu_summary_ = summary;
            appendHistory(cpu_history_, avg_usage / 100.0);
            cpu_core_rows_ = rows;
            emit cpuSummaryChanged();
            emit cpuHistoryChanged();
            emit cpuCoreRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateMemory(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString total_physical = valueForKey(status, "Total Memory (Physical)");
    const QString used_physical = valueForKey(status, "Used Memory (Physical)");
    const QString free_physical = valueForKey(status, "Free Memory (Physical)");
    const QString total_swap = valueForKey(status, "Total Memory (Swap)");
    const QString used_swap = valueForKey(status, "Used Memory (Swap)");
    const QString free_swap = valueForKey(status, "Free Memory (Swap)");
    const QString total_all = valueForKey(status, "Total Memory");
    const QString used_all = valueForKey(status, "Used Memory");
    const QString free_all = valueForKey(status, "Free Memory");

    const double total_physical_num = extractNumber(total_physical);
    const double used_physical_num = extractNumber(used_physical);
    const double usage_percent = total_physical_num > 0.0
                                     ? (used_physical_num / total_physical_num) * 100.0
                                     : 0.0;

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["usedPhysical"] = used_physical;
    summary["totalPhysical"] = total_physical;
    summary["freePhysical"] = free_physical;
    summary["usedSwap"] = used_swap;
    summary["totalSwap"] = total_swap;
    summary["usagePercent"] = formatNumber(usage_percent, 1) + "%";
    summary["updateStatus"] = valueForKey(status, "Update Status");

    QVariantList rows;
    rows.append(QVariantMap{{"item", "Physical"}, {"total", total_physical}, {"used", used_physical}, {"free", free_physical}});
    rows.append(QVariantMap{{"item", "Swap"}, {"total", total_swap}, {"used", used_swap}, {"free", free_swap}});
    rows.append(QVariantMap{{"item", "Combined"}, {"total", total_all}, {"used", used_all}, {"free", free_all}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, usage_percent]()
        {
            memory_summary_ = summary;
            appendHistory(memory_history_, usage_percent / 100.0);
            memory_rows_ = rows;
            emit memorySummaryChanged();
            emit memoryHistoryChanged();
            emit memoryRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateHdd(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_disk;
    bool has_disk = false;
    double worst_use = 0.0;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key.contains("Name") && key.startsWith("Disk")) {
            if (has_disk) {
                rows.append(current_disk);
            }
            current_disk.clear();
            current_disk["disk"] = value;
            has_disk = true;
        } else if (has_disk && key.contains("Size")) {
            current_disk["size"] = value;
        } else if (has_disk && key.contains("Available")) {
            current_disk["available"] = value;
        } else if (has_disk && key.contains("Use")) {
            current_disk["use"] = value;
            worst_use = std::max(worst_use, extractNumber(value));
        } else if (has_disk && key.contains("Status")) {
            current_disk["status"] = value;
        } else if (has_disk && key.contains("Mount Point")) {
            current_disk["mount"] = value;
        }
    }

    if (has_disk) {
        rows.append(current_disk);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["diskCount"] = static_cast<int>(rows.size());
    summary["worstUse"] = formatNumber(worst_use, 0) + "%";

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            hdd_summary_ = summary;
            hdd_rows_ = rows;
            emit hddSummaryChanged();
            emit hddRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNet(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_if;
    bool has_interface = false;
    double total_in = 0.0;
    double total_out = 0.0;
    int total_errors = 0;
    QStringList interfaces;
    QStringList ip_entries;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key == "Interface Name") {
            if (has_interface) {
                rows.append(current_if);
            }
            current_if.clear();
            current_if["interface"] = value;
            interfaces << value;
            has_interface = true;
        } else if (has_interface && key == "State") {
            current_if["state"] = value;
        } else if (has_interface && key == "IP Address") {
            current_if["ip"] = value;
            ip_entries << current_if.value("interface").toString() + ": " + value;
        } else if (has_interface && key == "Input Traffic") {
            current_if["input"] = value;
            total_in += extractNumber(value);
        } else if (has_interface && key == "Output Traffic") {
            current_if["output"] = value;
            total_out += extractNumber(value);
        } else if (has_interface && key == "MTU") {
            current_if["mtu"] = value;
        } else if (has_interface && key == "Total received MB") {
            current_if["totalRx"] = value;
        } else if (has_interface && key == "Total transmitted MB") {
            current_if["totalTx"] = value;
        } else if (has_interface && key == "Collisions") {
            current_if["collisions"] = value;
        } else if (has_interface && key == "Rx Errors") {
            current_if["rxErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        } else if (has_interface && key == "Tx Errors") {
            current_if["txErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        }
    }

    if (has_interface) {
        rows.append(current_if);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["input"] = formatNumber(total_in, 4) + " MB/s";
    summary["output"] = formatNumber(total_out, 4) + " MB/s";
    summary["interfaces"] = interfaces.join(", ");
    summary["ipAddresses"] = ip_entries.join("\n");
    summary["interfaceCount"] = interfaces.size();
    summary["errors"] = total_errors;

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, total_in, total_out]()
        {
            net_summary_ = summary;
            appendHistory(net_in_history_, total_in);
            appendHistory(net_out_history_, total_out);
            net_interface_rows_ = rows;
            emit netSummaryChanged();
            emit netInHistoryChanged();
            emit netOutHistoryChanged();
            emit netInterfaceRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNtp(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString offset = valueForKey(status, "Offset (us)");
    const QString tolerance = valueForKey(status, "Offset tolerance (us)");
    const QString error_tolerance = valueForKey(status, "Offset tolerance (us) for Error");

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["offset"] = offset + " us";
    summary["tolerance"] = tolerance + " us";
    summary["errorTolerance"] = error_tolerance + " us";

    QVariantList rows;
    rows.append(QVariantMap{{"name", "Offset (us)"}, {"value", offset}});
    rows.append(QVariantMap{{"name", "Tolerance (us)"}, {"value", tolerance}});
    rows.append(QVariantMap{{"name", "Error Tolerance (us)"}, {"value", error_tolerance}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            ntp_summary_ = summary;
            ntp_rows_ = rows;
            emit ntpSummaryChanged();
            emit ntpRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::handleVideoTopicsMessage(const std_msgs::msg::String::SharedPtr msg)
{
    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!document.isObject()) {
        return;
    }

    const QJsonArray topics = document.object().value(QStringLiteral("topics")).toArray();
    QStringList discovered_names;
    QMap<QString, QString> discovered_types;

    for (const QJsonValue &value : topics) {
        const QJsonObject topic_object = value.toObject();
        const QString name = topic_object.value(QStringLiteral("name")).toString().trimmed();
        const QString type = topic_object.value(QStringLiteral("type")).toString().trimmed();
        if (!isPerceptionImageTopic(name, type)) {
            continue;
        }

        discovered_names.append(name);
        discovered_types.insert(name, type);
    }

    QMetaObject::invokeMethod(
        this,
        [this, discovered_names, discovered_types]()
        {
            applyDiscoveredVideoTopics(discovered_names, discovered_types);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::applyDiscoveredVideoTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes)
{
    QStringList discovered_names = topicNames;
    discovered_names.removeDuplicates();
    discovered_names.sort(Qt::CaseSensitive);

    QMap<QString, QString> discovered_types;
    QVariantList topic_items;
    for (const QString &name : discovered_names) {
        const QString type = topicTypes.value(name);
        if (!isPerceptionImageTopic(name, type)) {
            continue;
        }
        discovered_types.insert(name, type);
        topic_items.append(name);
    }

    if (video_topic_names_ == discovered_names && video_topic_types_ == discovered_types) {
        return;
    }

    video_topic_names_ = discovered_names;
    video_topic_types_ = discovered_types;
    video_topics_ = topic_items;
    emit videoTopicsChanged();

    for (int slot_index = 0; slot_index < kVideoSlotCount; ++slot_index) {
        auto &slot = video_slots_[static_cast<size_t>(slot_index)];
        if (!slot.topic.isEmpty() && !video_topic_names_.contains(slot.topic)) {
            stopVideoSlot(slot_index);
            slot.topic.clear();
            slot.topic_type.clear();
            slot.status = QStringLiteral("No topic selected");
            emitVideoSlotChanged(slot_index);
        }
    }

}

bool RosUiBridge::isPerceptionImageTopic(const QString &topicName, const QString &topicType)
{
    return topicName.startsWith(QStringLiteral("/perception"))
           && (topicType == QString::fromLatin1(kImageType)
               || topicType == QString::fromLatin1(kCompressedImageType));
}

void RosUiBridge::handleVideoSlotStatusMessage(int slotIndex, const std_msgs::msg::String::SharedPtr msg)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QJsonDocument document = QJsonDocument::fromJson(QByteArray::fromStdString(msg->data));
    if (!document.isObject()) {
        return;
    }

    const QJsonObject object = document.object();
    const QString selected_topic = object.value(QStringLiteral("selected_topic")).toString().trimmed();
    const QString topic_type = object.value(QStringLiteral("topic_type")).toString().trimmed();
    const QString status = object.value(QStringLiteral("status")).toString().trimmed();

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, selected_topic, topic_type, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            slot.topic = selected_topic;
            slot.topic_type = topic_type;
            if (!status.isEmpty()) {
                slot.status = status;
            }
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::publishVideoSelection(int slotIndex, const QString &topicName)
{
    if (!video_select_pub_) {
        return;
    }

    QJsonObject payload;
    payload.insert(QStringLiteral("slot"), slotIndex);
    payload.insert(QStringLiteral("topic"), topicName);

    std_msgs::msg::String msg;
    msg.data = QJsonDocument(payload).toJson(QJsonDocument::Compact).toStdString();
    video_select_pub_->publish(msg);
}

void RosUiBridge::stopVideoSlot(int slotIndex)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    slot.image_sub.reset();
    slot.compressed_sub.reset();
    slot.frame_revision = 0;

    {
        std::lock_guard<std::mutex> lock(video_frame_mutex_);
        slot.frame = QImage();
    }

    if (slot.topic.isEmpty()) {
        slot.status = QStringLiteral("No topic selected");
    }
    emitVideoSlotChanged(slotIndex);
}

void RosUiBridge::startVideoSlot(int slotIndex)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
    slot.image_sub.reset();
    slot.compressed_sub.reset();

    if (slot.topic.isEmpty()) {
        slot.status = QStringLiteral("No topic selected");
        emitVideoSlotChanged(slotIndex);
        return;
    }

    const std::string selected_topic = slot.topic.toStdString();
    const rclcpp::SensorDataQoS qos;

    slot.status = QStringLiteral("Waiting for video frame");

    if (slot.topic_type == QString::fromLatin1(kImageType) || slot.topic_type.isEmpty()) {
        slot.image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
            selected_topic, qos,
            [this, slotIndex](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                imageCallback(slotIndex, msg);
            });
    }

    if (slot.topic_type == QString::fromLatin1(kCompressedImageType) || slot.topic_type.isEmpty()) {
        slot.compressed_sub = node_->create_subscription<sensor_msgs::msg::CompressedImage>(
            selected_topic, qos,
            [this, slotIndex](const sensor_msgs::msg::CompressedImage::SharedPtr msg)
            {
                compressedImageCallback(slotIndex, msg);
            });
    }

    emitVideoSlotChanged(slotIndex);
}

void RosUiBridge::imageCallback(int slotIndex, const sensor_msgs::msg::Image::SharedPtr msg)
{
    QString error;
    const QImage image = imageMessageToQImage(*msg, &error);
    if (image.isNull()) {
        updateVideoStatus(slotIndex, error.isEmpty() ? QStringLiteral("Unable to decode image") : error);
        return;
    }

    const QString status = QStringLiteral("%1x%2 %3")
                               .arg(msg->width)
                               .arg(msg->height)
                               .arg(QString::fromStdString(msg->encoding));
    updateVideoFrame(slotIndex, image, status);
}

void RosUiBridge::compressedImageCallback(int slotIndex, const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    QImage image;
    if (!image.loadFromData(msg->data.data(), static_cast<int>(msg->data.size()))) {
        updateVideoStatus(slotIndex, QStringLiteral("Unable to decode compressed image"));
        return;
    }

    const QString format = QString::fromStdString(msg->format).trimmed();
    const QString status = format.isEmpty()
                               ? QStringLiteral("Compressed image")
                               : QStringLiteral("Compressed image (%1)").arg(format);
    updateVideoFrame(slotIndex, image, status);
}

void RosUiBridge::updateVideoFrame(int slotIndex, const QImage &image, const QString &status)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    const QImage image_copy = image.copy();

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, image_copy, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            {
                std::lock_guard<std::mutex> lock(video_frame_mutex_);
                slot.frame = image_copy;
            }

            ++slot.frame_revision;
            slot.status = status;
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateVideoStatus(int slotIndex, const QString &status)
{
    if (slotIndex < 0 || slotIndex >= kVideoSlotCount) {
        return;
    }

    QMetaObject::invokeMethod(
        this,
        [this, slotIndex, status]()
        {
            auto &slot = video_slots_[static_cast<size_t>(slotIndex)];
            slot.status = status;
            emitVideoSlotChanged(slotIndex);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::emitVideoSlotChanged(int slotIndex)
{
    if (slotIndex == 0) {
        emit videoSlot0Changed();
    } else if (slotIndex == 1) {
        emit videoSlot1Changed();
    } else if (slotIndex == 2) {
        emit videoSlot2Changed();
    } else if (slotIndex == 3) {
        emit videoSlot3Changed();
    }

    emit videoSlotsChanged();
}

QVariantList RosUiBridge::plotFieldOptionsForTopic(const QString &topicName, const QString &topicType)
{
    QVariantList fields;

    auto append_scalar = [&fields, &topicName, &topicType](const QString &unit = QString())
    {
        appendPlotField(fields, topicName, topicType, QString(), unit);
    };

    auto append_vector3 = [&fields, &topicName, &topicType](const QString &prefix, const QString &unit)
    {
        appendPlotField(fields, topicName, topicType, prefix.isEmpty() ? QStringLiteral("x") : QStringLiteral("%1.x").arg(prefix), unit);
        appendPlotField(fields, topicName, topicType, prefix.isEmpty() ? QStringLiteral("y") : QStringLiteral("%1.y").arg(prefix), unit);
        appendPlotField(fields, topicName, topicType, prefix.isEmpty() ? QStringLiteral("z") : QStringLiteral("%1.z").arg(prefix), unit);
    };

    if (topicType == QStringLiteral("std_msgs/msg/Bool")) {
        append_scalar();
    } else if (topicType == QStringLiteral("std_msgs/msg/Float32")
               || topicType == QStringLiteral("std_msgs/msg/Float64")
               || topicType == QStringLiteral("std_msgs/msg/Int8")
               || topicType == QStringLiteral("std_msgs/msg/Int16")
               || topicType == QStringLiteral("std_msgs/msg/Int32")
               || topicType == QStringLiteral("std_msgs/msg/Int64")
               || topicType == QStringLiteral("std_msgs/msg/UInt8")
               || topicType == QStringLiteral("std_msgs/msg/UInt16")
               || topicType == QStringLiteral("std_msgs/msg/UInt32")
               || topicType == QStringLiteral("std_msgs/msg/UInt64")) {
        append_scalar();
    } else if (topicType == QStringLiteral("sensor_msgs/msg/Imu")) {
        append_vector3(QStringLiteral("angular_velocity"), QStringLiteral("rad/s"));
        append_vector3(QStringLiteral("linear_acceleration"), QStringLiteral("m/s^2"));
    } else if (topicType == QStringLiteral("sensor_msgs/msg/Temperature")) {
        appendPlotField(fields, topicName, topicType, QStringLiteral("temperature"), QStringLiteral("deg C"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("variance"), QStringLiteral("deg C^2"));
    } else if (topicType == QStringLiteral("sensor_msgs/msg/FluidPressure")) {
        appendPlotField(fields, topicName, topicType, QStringLiteral("fluid_pressure"), QStringLiteral("Pa"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("variance"), QStringLiteral("Pa^2"));
    } else if (topicType == QStringLiteral("sensor_msgs/msg/RelativeHumidity")) {
        appendPlotField(fields, topicName, topicType, QStringLiteral("relative_humidity"), QStringLiteral("ratio"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("variance"), QStringLiteral("ratio^2"));
    } else if (topicType == QStringLiteral("sensor_msgs/msg/MagneticField")) {
        append_vector3(QStringLiteral("magnetic_field"), QStringLiteral("T"));
    } else if (topicType == QStringLiteral("sensor_msgs/msg/BatteryState")) {
        appendPlotField(fields, topicName, topicType, QStringLiteral("voltage"), QStringLiteral("V"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("temperature"), QStringLiteral("deg C"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("current"), QStringLiteral("A"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("charge"), QStringLiteral("Ah"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("capacity"), QStringLiteral("Ah"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("design_capacity"), QStringLiteral("Ah"));
        appendPlotField(fields, topicName, topicType, QStringLiteral("percentage"), QStringLiteral("ratio"));
    } else if (topicType == QStringLiteral("geometry_msgs/msg/Vector3")
               || topicType == QStringLiteral("geometry_msgs/msg/Vector3Stamped")) {
        append_vector3(QString(), QString());
    } else if (topicType == QStringLiteral("geometry_msgs/msg/Twist")
               || topicType == QStringLiteral("geometry_msgs/msg/TwistStamped")) {
        append_vector3(QStringLiteral("linear"), QStringLiteral("m/s"));
        append_vector3(QStringLiteral("angular"), QStringLiteral("rad/s"));
    } else if (topicType == QStringLiteral("geometry_msgs/msg/Accel")
               || topicType == QStringLiteral("geometry_msgs/msg/AccelStamped")) {
        append_vector3(QStringLiteral("linear"), QStringLiteral("m/s^2"));
        append_vector3(QStringLiteral("angular"), QStringLiteral("rad/s^2"));
    }

    return fields;
}

QString RosUiBridge::plotFieldPath(const QString &topicName, const QString &fieldName)
{
    return plotFieldPathForTopic(topicName, fieldName);
}

bool RosUiBridge::isSupportedPlotType(const QString &topicType)
{
    static const QStringList supported_types = {
        QStringLiteral("std_msgs/msg/Bool"),
        QStringLiteral("std_msgs/msg/Float32"),
        QStringLiteral("std_msgs/msg/Float64"),
        QStringLiteral("std_msgs/msg/Int8"),
        QStringLiteral("std_msgs/msg/Int16"),
        QStringLiteral("std_msgs/msg/Int32"),
        QStringLiteral("std_msgs/msg/Int64"),
        QStringLiteral("std_msgs/msg/UInt8"),
        QStringLiteral("std_msgs/msg/UInt16"),
        QStringLiteral("std_msgs/msg/UInt32"),
        QStringLiteral("std_msgs/msg/UInt64"),
        QStringLiteral("sensor_msgs/msg/Imu"),
        QStringLiteral("sensor_msgs/msg/Temperature"),
        QStringLiteral("sensor_msgs/msg/FluidPressure"),
        QStringLiteral("sensor_msgs/msg/RelativeHumidity"),
        QStringLiteral("sensor_msgs/msg/MagneticField"),
        QStringLiteral("sensor_msgs/msg/BatteryState"),
        QStringLiteral("geometry_msgs/msg/Vector3"),
        QStringLiteral("geometry_msgs/msg/Vector3Stamped"),
        QStringLiteral("geometry_msgs/msg/Twist"),
        QStringLiteral("geometry_msgs/msg/TwistStamped"),
        QStringLiteral("geometry_msgs/msg/Accel"),
        QStringLiteral("geometry_msgs/msg/AccelStamped")
    };

    return supported_types.contains(topicType);
}

QImage RosUiBridge::imageMessageToQImage(const sensor_msgs::msg::Image &msg, QString *errorMessage)
{
    if (msg.width == 0 || msg.height == 0) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Invalid image size");
        }
        return {};
    }

    const qsizetype required_size = static_cast<qsizetype>(msg.step) * static_cast<qsizetype>(msg.height);
    if (required_size <= 0 || static_cast<qsizetype>(msg.data.size()) < required_size) {
        if (errorMessage) {
            *errorMessage = QStringLiteral("Image data is incomplete");
        }
        return {};
    }

    const uchar *data = msg.data.data();
    const int width = static_cast<int>(msg.width);
    const int height = static_cast<int>(msg.height);
    const int bytes_per_line = static_cast<int>(msg.step);
    const QString encoding = QString::fromStdString(msg.encoding).toLower();

    if (encoding == QStringLiteral("rgb8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGB888).copy();
    }

    if (encoding == QStringLiteral("bgr8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGB888).rgbSwapped();
    }

    if (encoding == QStringLiteral("rgba8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGBA8888).copy();
    }

    if (encoding == QStringLiteral("bgra8")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_RGBA8888).rgbSwapped();
    }

    if (encoding == QStringLiteral("mono8") || encoding == QStringLiteral("8uc1")) {
        return QImage(data, width, height, bytes_per_line, QImage::Format_Grayscale8).copy();
    }

    if (errorMessage) {
        *errorMessage = QStringLiteral("Unsupported image encoding: %1").arg(QString::fromStdString(msg.encoding));
    }
    return {};
}
