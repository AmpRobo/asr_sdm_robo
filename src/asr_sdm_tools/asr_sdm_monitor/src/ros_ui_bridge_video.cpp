#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <Qt>

namespace
{
constexpr int kVideoSlotCount = 4;
constexpr const char *kImageType = "sensor_msgs/msg/Image";
constexpr const char *kCompressedImageType = "sensor_msgs/msg/CompressedImage";
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
    const bool is_supported_prefix = topicName.startsWith(QStringLiteral("/perception"))
                                     || topicName.startsWith(QStringLiteral("/sensing"));
    return is_supported_prefix
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

    if (!video_node_) {
        slot.status = QStringLiteral("Video ROS node unavailable");
        emitVideoSlotChanged(slotIndex);
        return;
    }

    if (slot.topic.isEmpty()) {
        slot.status = QStringLiteral("No topic selected");
        emitVideoSlotChanged(slotIndex);
        return;
    }

    const std::string selected_topic = slot.topic.toStdString();
    const rclcpp::SensorDataQoS qos;

    slot.status = QStringLiteral("Waiting for video frame");

    if (slot.topic_type == QString::fromLatin1(kImageType) || slot.topic_type.isEmpty()) {
        slot.image_sub = video_node_->create_subscription<sensor_msgs::msg::Image>(
            selected_topic, qos,
            [this, slotIndex](const sensor_msgs::msg::Image::SharedPtr msg)
            {
                imageCallback(slotIndex, msg);
            });
    }

    if (slot.topic_type == QString::fromLatin1(kCompressedImageType) || slot.topic_type.isEmpty()) {
        slot.compressed_sub = video_node_->create_subscription<sensor_msgs::msg::CompressedImage>(
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
