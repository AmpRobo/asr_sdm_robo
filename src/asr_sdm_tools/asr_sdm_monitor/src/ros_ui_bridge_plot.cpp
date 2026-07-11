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

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/message_info.hpp>
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
constexpr int kMaxPlotSamples = 600;

QString normalizedNodeFullName(const QString &nodeName, const QString &nodeNamespace)
{
    QString ns = nodeNamespace.trimmed();
    QString name = nodeName.trimmed();
    if (name.isEmpty()) {
        return ns;
    }
    if (ns.isEmpty() || ns == QStringLiteral("/")) {
        return QStringLiteral("/") + name;
    }
    if (!ns.startsWith(QLatin1Char('/'))) {
        ns.prepend(QLatin1Char('/'));
    }
    if (ns.endsWith(QLatin1Char('/'))) {
        ns.chop(1);
    }
    return ns + QStringLiteral("/") + name;
}

QByteArray endpointGidBytes(const std::array<uint8_t, RMW_GID_STORAGE_SIZE> &gid)
{
    return QByteArray(
        reinterpret_cast<const char *>(gid.data()),
        static_cast<int>(gid.size()));
}

QByteArray publisherGidBytes(const rclcpp::MessageInfo &messageInfo)
{
    const auto &gid = messageInfo.get_rmw_message_info().publisher_gid;
    return QByteArray(
        reinterpret_cast<const char *>(gid.data),
        static_cast<int>(RMW_GID_STORAGE_SIZE));
}

bool gidHasNonZeroByte(const QByteArray &gid)
{
    for (const char byte : gid) {
        if (byte != 0) {
            return true;
        }
    }
    return false;
}

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
    sample->insert(QStringLiteral("sourceTopic"), topicName);
    return true;
}
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
    std::lock_guard<std::mutex> lock(plot_recording_mutex_);
    return plot_recording_;
}

QString RosUiBridge::plotRecordingPath() const
{
    std::lock_guard<std::mutex> lock(plot_recording_mutex_);
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

    // The Live/Recorded switch controls only which samples and field list are
    // drawn. The Topics model is unified and never changes with this switch.
    plot_data_source_ = next;
    rebuildPlotFieldOptions();
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

        // Recording is independent from plotting: every selected active ROS
        // graph source is subscribed in serialized form, including
        // non-plottable message types.
        refreshRecordSubscriptions();

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

    // Remove serialized subscriptions first so no new callback can enter the
    // writer after it is closed.
    stopRecordSubscriptions();

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

void RosUiBridge::writeSerializedRecordingSample(
    const QString &topicName,
    const QString &topicType,
    const std::shared_ptr<rclcpp::SerializedMessage> &message)
{
    if (!message) {
        return;
    }

    std::lock_guard<std::mutex> lock(plot_recording_mutex_);
    if (!plot_recording_ || !plot_bag_writer_) {
        return;
    }

    try {
        const rclcpp::Time timestamp = plot_node_
                                           ? plot_node_->now()
                                           : rclcpp::Time(static_cast<int64_t>(fallbackNowMs() * 1000000.0));
        plot_bag_writer_->write(
            message,
            topicName.toStdString(),
            topicType.toStdString(),
            timestamp);
        ++plot_recorded_message_count_;
    } catch (const std::exception &error) {
        plot_recording_ = false;
        plot_bag_writer_.reset();
        const QString errorStatus =
            QStringLiteral("Bag recording stopped: %1").arg(QString::fromUtf8(error.what()));
        QMetaObject::invokeMethod(
            this,
            [this, errorStatus]()
            {
                stopRecordSubscriptions();
                plot_status_ = errorStatus;
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

        QStringList bagTopicNames;
        QMap<QString, QString> bagTopicTypes;
        QVariantList availableFields;
        const auto topicsAndTypes = reader.get_all_topics_and_types();
        for (const auto &topicMetadata : topicsAndTypes) {
            const QString topicName = QString::fromStdString(topicMetadata.name).trimmed();
            const QString topicType = QString::fromStdString(topicMetadata.type).trimmed();
            if (topicName.isEmpty() || topicType.isEmpty()) {
                continue;
            }

            bagTopicNames.append(topicName);
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

        bagTopicNames.removeDuplicates();
        bagTopicNames.sort(Qt::CaseSensitive);

        const bool sameBag = recorded_file_path_ == filePath;
        if (!sameBag) {
            selected_recorded_plot_topic_names_.clear();
        }

        recorded_file_path_ = filePath;
        recorded_topic_source_name_ = QFileInfo(filePath).fileName();
        if (recorded_topic_source_name_.isEmpty()) {
            recorded_topic_source_name_ = filePath;
        }
        recorded_plot_topic_names_ = bagTopicNames;
        recorded_plot_topic_types_ = bagTopicTypes;
        recorded_all_plot_samples_ = samples;
        recorded_available_plot_field_options_ = availableFields;

        // Recorded plotting is intentionally independent of the Topics-page
        // checkboxes. Opening a bag immediately exposes every plottable field
        // in the Recorded controls and makes all decoded samples available to
        // the existing playback/curve logic, matching the previous package.
        recorded_plot_field_options_ = recorded_available_plot_field_options_;
        recorded_plot_samples_ = recorded_all_plot_samples_;
        recorded_bag_start_time_ms_ = bagStartMs;
        recorded_bag_end_time_ms_ = bagEndMs;
        playback_start_time_ms_ = bagStartMs;
        playback_end_time_ms_ = bagEndMs;
        playback_current_time_ms_ = playback_start_time_ms_;
        recorded_status_ = samples.isEmpty()
                               ? QStringLiteral("Loaded bag: %1; no supported plot topics found").arg(filePath)
                               : QStringLiteral("Loaded bag: %1 (%2 plot samples)").arg(filePath).arg(samples.size());

        rebuildPlotFieldOptions();
        rebuildPlotTopicsModel();
        return true;
    } catch (const std::exception &error) {
        recorded_status_ = QStringLiteral("Cannot load rosbag / MCAP: %1").arg(QString::fromUtf8(error.what()));
        return false;
    }
}

void RosUiBridge::rebuildRecordedPlotSamples()
{
    // Recorded field selection is performed inside the Recorded page itself.
    // Topics-page selection must not hide bag samples or disable playback.
    recorded_plot_samples_ = recorded_all_plot_samples_;
    emit recordedPlotSamplesChanged();
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

QString RosUiBridge::recordedTopicKey(const QString &topicName) const
{
    return QStringLiteral("bag|%1|%2").arg(recorded_file_path_, topicName);
}

const RosUiBridge::PlotTopicSource *RosUiBridge::graphTopicSourceForKey(const QString &key) const
{
    for (const PlotTopicSource &source : graph_plot_topic_sources_) {
        if (source.key == key) {
            return &source;
        }
    }
    return nullptr;
}

QStringList RosUiBridge::selectedGraphTopicNames() const
{
    QStringList topics;
    for (const QString &key : selected_graph_plot_source_keys_) {
        const PlotTopicSource *source = graphTopicSourceForKey(key);
        if (source && !source->topic_name.isEmpty()) {
            topics.append(source->topic_name);
        }
    }
    topics.removeDuplicates();
    topics.sort(Qt::CaseSensitive);
    return topics;
}

bool RosUiBridge::messageMatchesSelectedGraphSource(
    const QString &topicName,
    const rclcpp::MessageInfo &messageInfo) const
{
    std::lock_guard<std::mutex> sourceLock(plot_source_mutex_);

    const QByteArray publisherGid = publisherGidBytes(messageInfo);
    const bool gidAvailable = gidHasNonZeroByte(publisherGid);
    int availableSourceCount = 0;
    int selectedSourceCount = 0;
    bool selectedGidMatched = false;

    for (const PlotTopicSource &source : graph_plot_topic_sources_) {
        if (source.topic_name != topicName) {
            continue;
        }

        ++availableSourceCount;
        if (!selected_graph_plot_source_keys_.contains(source.key)) {
            continue;
        }

        ++selectedSourceCount;
        if (gidAvailable && source.publisher_gids.contains(publisherGid)) {
            selectedGidMatched = true;
        }
    }

    if (selectedSourceCount == 0) {
        return false;
    }

    // GID filtering is only needed when the same topic currently has multiple
    // independently selectable sources and only a subset of them is selected.
    // For the common case of one source, or when every source is selected,
    // accepting directly avoids RMW-specific endpoint-GID representation
    // differences from incorrectly dropping every Plot/Record message.
    if (availableSourceCount == 1 || selectedSourceCount == availableSourceCount) {
        return true;
    }

    return gidAvailable && selectedGidMatched;
}

bool RosUiBridge::isRosbagPlayerNode(const QString &nodeName, const QString &nodeNamespace) const
{
    const QString lowerName = nodeName.trimmed().toLower();
    if (lowerName.contains(QStringLiteral("rosbag2_player"))) {
        return true;
    }

    if (!node_) {
        return false;
    }

    try {
        const auto services = node_->get_service_names_and_types_by_node(
            nodeName.toStdString(), nodeNamespace.toStdString());
        int rosbagControlServiceCount = 0;
        for (const auto &entry : services) {
            const QString serviceName = QString::fromStdString(entry.first).toLower();
            bool hasRosbagType = false;
            for (const std::string &type : entry.second) {
                if (QString::fromStdString(type).startsWith(QStringLiteral("rosbag2_interfaces/srv/"))) {
                    hasRosbagType = true;
                    break;
                }
            }
            if (!hasRosbagType) {
                continue;
            }

            if (serviceName.endsWith(QStringLiteral("/pause"))
                || serviceName.endsWith(QStringLiteral("/resume"))
                || serviceName.endsWith(QStringLiteral("/seek"))
                || serviceName.endsWith(QStringLiteral("/set_rate"))
                || serviceName.endsWith(QStringLiteral("/play_next"))
                || serviceName.endsWith(QStringLiteral("/burst"))) {
                ++rosbagControlServiceCount;
            }
        }
        return rosbagControlServiceCount >= 2;
    } catch (const std::exception &) {
        return false;
    }
}

void RosUiBridge::setPlotTopicSelected(const QString &topicKey, bool selected)
{
    const QString normalizedKey = topicKey.trimmed();
    if (normalizedKey.isEmpty()) {
        return;
    }

    bool graphSourceSelected = false;
    {
        std::lock_guard<std::mutex> sourceLock(plot_source_mutex_);
        graphSourceSelected = graphTopicSourceForKey(normalizedKey) != nullptr;
        if (graphSourceSelected) {
            const bool alreadySelected = selected_graph_plot_source_keys_.contains(normalizedKey);
            if (selected == alreadySelected) {
                return;
            }

            if (selected) {
                const PlotTopicSource *selectedSource = graphTopicSourceForKey(normalizedKey);
                if (!selectedSource) {
                    return;
                }

                // A topic name may be published by both a live node and a
                // rosbag2 player. Only one source may be selected for the same
                // topic name, while different topic names remain independently
                // selectable.
                const QString selectedTopicName = selectedSource->topic_name;
                for (const PlotTopicSource &source : graph_plot_topic_sources_) {
                    if (source.topic_name == selectedTopicName) {
                        selected_graph_plot_source_keys_.removeAll(source.key);
                    }
                }
                selected_graph_plot_source_keys_.append(normalizedKey);
                selected_graph_plot_source_keys_.removeDuplicates();
                selected_graph_plot_source_keys_.sort(Qt::CaseSensitive);
            } else {
                selected_graph_plot_source_keys_.removeAll(normalizedKey);
            }
        }
    }

    if (graphSourceSelected) {
        {
            std::lock_guard<std::mutex> lock(live_plot_mutex_);
            pending_live_plot_samples_.clear();
            pending_live_plot_status_topic_.clear();
        }
        imu_plot_samples_.clear();
        plot_start_time_ = -1.0;
        refreshPlotSubscriptions();
        if (plotRecording()) {
            refreshRecordSubscriptions();
        }
        emit imuPlotSamplesChanged();
        rebuildPlotTopicsModel();
        rebuildPlotFieldOptions();
        return;
    }

    // Offline bag topics are selected directly inside Recorded after Open.

}

void RosUiBridge::discoverPlotTopics()
{
    if (!node_) {
        return;
    }

    QList<PlotTopicSource> discoveredSources;
    QMap<QString, int> sourceIndexByKey;
    QMap<QString, QStringList> publisherNodesByKey;
    QMap<QString, bool> playerNodeCache;

    const auto topicsAndTypes = node_->get_topic_names_and_types();
    for (const auto &entry : topicsAndTypes) {
        const QString topicName = QString::fromStdString(entry.first).trimmed();
        if (topicName.isEmpty()) {
            continue;
        }

        std::vector<rclcpp::TopicEndpointInfo> publishers;
        try {
            publishers = node_->get_publishers_info_by_topic(entry.first);
        } catch (const std::exception &) {
            continue;
        }

        for (const rclcpp::TopicEndpointInfo &publisher : publishers) {
            const QString nodeName = QString::fromStdString(publisher.node_name()).trimmed();
            const QString nodeNamespace = QString::fromStdString(publisher.node_namespace()).trimmed();
            const QString fullNodeName = normalizedNodeFullName(nodeName, nodeNamespace);
            const QString cacheKey = nodeNamespace + QStringLiteral("|") + nodeName;
            if (!playerNodeCache.contains(cacheKey)) {
                playerNodeCache.insert(cacheKey, isRosbagPlayerNode(nodeName, nodeNamespace));
            }

            const bool bagPlay = playerNodeCache.value(cacheKey);
            const QString sourceKind = bagPlay
                                         ? QStringLiteral("ros2_bag_play")
                                         : QStringLiteral("ros2_live");
            const QString sourceKey = QStringLiteral("graph|%1|%2").arg(sourceKind, topicName);
            QString topicType = QString::fromStdString(publisher.topic_type()).trimmed();
            if (topicType.isEmpty() && !entry.second.empty()) {
                topicType = QString::fromStdString(entry.second.front()).trimmed();
            }

            int sourceIndex = sourceIndexByKey.value(sourceKey, -1);
            if (sourceIndex < 0) {
                PlotTopicSource source;
                source.key = sourceKey;
                source.topic_name = topicName;
                source.topic_type = topicType;
                source.source_kind = sourceKind;
                discoveredSources.append(source);
                sourceIndex = discoveredSources.size() - 1;
                sourceIndexByKey.insert(sourceKey, sourceIndex);
            }

            PlotTopicSource &source = discoveredSources[sourceIndex];
            const QByteArray gid = endpointGidBytes(publisher.endpoint_gid());
            if (gidHasNonZeroByte(gid) && !source.publisher_gids.contains(gid)) {
                source.publisher_gids.append(gid);
            }
            QStringList &publisherNodes = publisherNodesByKey[sourceKey];
            if (!fullNodeName.isEmpty() && !publisherNodes.contains(fullNodeName)) {
                publisherNodes.append(fullNodeName);
            }
        }
    }

    for (PlotTopicSource &source : discoveredSources) {
        std::sort(source.publisher_gids.begin(), source.publisher_gids.end());
        QStringList publisherNodes = publisherNodesByKey.value(source.key);
        publisherNodes.sort(Qt::CaseSensitive);
        source.source_name = publisherNodes.join(QStringLiteral(", "));
    }

    std::sort(discoveredSources.begin(), discoveredSources.end(),
              [](const PlotTopicSource &left, const PlotTopicSource &right)
              {
                  if (left.topic_name != right.topic_name) {
                      return left.topic_name < right.topic_name;
                  }
                  if (left.source_kind != right.source_kind) {
                      return left.source_kind < right.source_kind;
                  }
                  return left.key < right.key;
              });

    QMetaObject::invokeMethod(
        this,
        [this, discoveredSources]()
        {
            applyDiscoveredPlotTopics(discoveredSources);
        },
        Qt::QueuedConnection);
}

void RosUiBridge::applyDiscoveredPlotTopics(const QList<PlotTopicSource> &sources)
{
    bool topicGraphChanged = false;
    bool selectionChanged = false;
    {
        std::lock_guard<std::mutex> sourceLock(plot_source_mutex_);
        topicGraphChanged = graph_plot_topic_sources_ != sources;
        graph_plot_topic_sources_ = sources;

        live_plot_topic_types_.clear();
        QStringList validSourceKeys;
        for (const PlotTopicSource &source : graph_plot_topic_sources_) {
            validSourceKeys.append(source.key);
            if (!source.topic_name.isEmpty() && !source.topic_type.isEmpty()) {
                live_plot_topic_types_.insert(source.topic_name, source.topic_type);
            }
        }

        QStringList retainedSelection;
        QStringList retainedTopicNames;
        for (const QString &key : selected_graph_plot_source_keys_) {
            if (!validSourceKeys.contains(key)) {
                continue;
            }
            const PlotTopicSource *source = graphTopicSourceForKey(key);
            if (!source || retainedTopicNames.contains(source->topic_name)) {
                continue;
            }
            retainedSelection.append(key);
            retainedTopicNames.append(source->topic_name);
        }
        selectionChanged = retainedSelection != selected_graph_plot_source_keys_;
        selected_graph_plot_source_keys_ = retainedSelection;
    }

    rebuildPlotTopicsModel();

    if (topicGraphChanged || selectionChanged) {
        rebuildPlotFieldOptions();
        refreshPlotSubscriptions();
        if (plotRecording()) {
            refreshRecordSubscriptions();
        }
    }
}

void RosUiBridge::rebuildPlotTopicsModel()
{
    QVariantList topicItems;
    int selectedCount = 0;
    int plottableFieldCount = 0;

    for (const PlotTopicSource &source : graph_plot_topic_sources_) {
        const QVariantList fields = plotFieldOptionsForTopic(source.topic_name, source.topic_type);
        const bool selected = selected_graph_plot_source_keys_.contains(source.key);
        if (selected) {
            ++selectedCount;
            plottableFieldCount += fields.size();
        }

        QVariantMap item;
        item[QStringLiteral("key")] = source.key;
        item[QStringLiteral("name")] = source.topic_name;
        item[QStringLiteral("type")] = source.topic_type;
        item[QStringLiteral("selected")] = selected;
        item[QStringLiteral("plottable")] = !fields.isEmpty();
        item[QStringLiteral("fieldCount")] = fields.size();
        item[QStringLiteral("sourceKind")] = source.source_kind;
        item[QStringLiteral("sourceName")] = source.source_name;
        item[QStringLiteral("recordable")] = true;
        topicItems.append(item);
    }


    auto sourceRank = [](const QString &kind)
    {
        if (kind == QStringLiteral("ros2_live")) {
            return 0;
        }
        if (kind == QStringLiteral("ros2_bag_play")) {
            return 1;
        }
        return 2;
    };
    std::sort(topicItems.begin(), topicItems.end(),
              [&sourceRank](const QVariant &leftValue, const QVariant &rightValue)
              {
                  const QVariantMap left = leftValue.toMap();
                  const QVariantMap right = rightValue.toMap();
                  const QString leftName = left.value(QStringLiteral("name")).toString();
                  const QString rightName = right.value(QStringLiteral("name")).toString();
                  if (leftName != rightName) {
                      return leftName < rightName;
                  }
                  const int leftRank = sourceRank(left.value(QStringLiteral("sourceKind")).toString());
                  const int rightRank = sourceRank(right.value(QStringLiteral("sourceKind")).toString());
                  if (leftRank != rightRank) {
                      return leftRank < rightRank;
                  }
                  return left.value(QStringLiteral("key")).toString()
                         < right.value(QStringLiteral("key")).toString();
              });

    plot_topics_ = topicItems;
    plot_topics_status_ = QStringLiteral("%1 topic sources, %2 selected, %3 plottable fields")
                              .arg(topicItems.size())
                              .arg(selectedCount)
                              .arg(plottableFieldCount);
    emit plotTopicsChanged();
}

QVariantList RosUiBridge::filterPlotFieldOptionsForSelectedTopics(
    const QVariantList &fields,
    const QStringList &selectedTopicNames)
{
    QVariantList result;
    for (const QVariant &candidate : fields) {
        const QVariantMap field = candidate.toMap();
        const QString topic = field.value(QStringLiteral("topic")).toString();
        if (!topic.isEmpty() && selectedTopicNames.contains(topic)) {
            result.append(field);
        }
    }
    return result;
}

void RosUiBridge::rebuildPlotFieldOptions()
{
    QVariantList liveFields;
    for (const QString &topic : selectedGraphTopicNames()) {
        const QString type = live_plot_topic_types_.value(topic);
        const QVariantList topicFields = plotFieldOptionsForTopic(topic, type);
        for (const QVariant &field : topicFields) {
            liveFields.append(field);
        }
    }

    plot_field_options_ = liveFields;

    // The Recorded page directly selects from every field decoded from the
    // opened bag. It does not depend on the unified Topics-page checkboxes.
    recorded_plot_field_options_ = recorded_available_plot_field_options_;
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

    if (!plot_node_) {
        return;
    }

    for (const QString &topic : selectedGraphTopicNames()) {
        const QString type = live_plot_topic_types_.value(topic);
        if (isSupportedPlotType(type)) {
            startPlotSubscriptionForTopic(topic, type);
        }
    }
}

void RosUiBridge::stopRecordSubscriptions()
{
    record_subscriptions_.clear();
}

void RosUiBridge::refreshRecordSubscriptions()
{
    stopRecordSubscriptions();

    bool recording = false;
    {
        std::lock_guard<std::mutex> lock(plot_recording_mutex_);
        recording = plot_recording_;
    }
    if (!plot_node_ || !recording) {
        return;
    }

    for (const QString &topic : selectedGraphTopicNames()) {
        const QString type = live_plot_topic_types_.value(topic).trimmed();
        if (!type.isEmpty()) {
            startRecordSubscriptionForTopic(topic, type);
        }
    }
}

void RosUiBridge::startRecordSubscriptionForTopic(const QString &topicName, const QString &topicType)
{
    if (!plot_node_ || topicName.isEmpty() || topicType.isEmpty()) {
        return;
    }

    try {
        auto subscription = plot_node_->create_generic_subscription(
            topicName.toStdString(),
            topicType.toStdString(),
            rclcpp::SensorDataQoS(),
            [this, topicName, topicType](
                std::shared_ptr<rclcpp::SerializedMessage> message,
                const rclcpp::MessageInfo &messageInfo)
            {
                if (!messageMatchesSelectedGraphSource(topicName, messageInfo)) {
                    return;
                }
                writeSerializedRecordingSample(topicName, topicType, message);
            });

        record_subscriptions_.insert(
            topicName,
            std::static_pointer_cast<rclcpp::SubscriptionBase>(subscription));
    } catch (const std::exception &error) {
        plot_status_ = QStringLiteral("Cannot record topic %1: %2")
                           .arg(topicName, QString::fromUtf8(error.what()));
        emit plotStatusChanged();
    }
}

template<typename MessageT>
void RosUiBridge::startTypedPlotSubscription(const QString &topicName)
{
    const std::string topic = topicName.toStdString();
    auto subscription = plot_node_->create_subscription<MessageT>(
        topic, rclcpp::SensorDataQoS(),
        [this, topicName](
            typename MessageT::SharedPtr msg,
            const rclcpp::MessageInfo &messageInfo)
        {
            if (!msg || !messageMatchesSelectedGraphSource(topicName, messageInfo)) {
                return;
            }

            const double fallbackAbsoluteTimeMs = plot_node_
                                                   ? static_cast<double>(plot_node_->now().nanoseconds()) / 1000000.0
                                                   : fallbackNowMs();
            const QVariantMap sample = sampleFromPlotMessage(topicName, *msg, fallbackAbsoluteTimeMs);
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
