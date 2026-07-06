#include "asr_sdm_monitor/ros_video_image_provider.hpp"

#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QStringList>

RosVideoImageProvider::RosVideoImageProvider(RosUiBridge *bridge)
    : QQuickImageProvider(QQuickImageProvider::Image),
      bridge_(bridge)
{
}

QImage RosVideoImageProvider::requestImage(const QString &id, QSize *size, const QSize &requestedSize)
{
    int slot_index = 0;
    const QString slot_token = id.split('/').value(0);
    bool ok = false;
    slot_index = slot_token.toInt(&ok);
    if (!ok) {
        slot_index = slot_token.startsWith(QStringLiteral("slot"))
                         ? slot_token.mid(4).toInt(&ok)
                         : 0;
        if (!ok) {
            slot_index = 0;
        }
    }

    QImage image;
    if (bridge_) {
        image = bridge_->videoFrameImage(slot_index);
    }

    if (size) {
        *size = image.size();
    }

    if (!image.isNull() && requestedSize.isValid()) {
        return image.scaled(requestedSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);
    }

    return image;
}
