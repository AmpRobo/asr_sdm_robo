#pragma once

#include <QImage>
#include <QQuickImageProvider>

class RosUiBridge;

class RosVideoImageProvider : public QQuickImageProvider
{
public:
    explicit RosVideoImageProvider(RosUiBridge *bridge);

    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;

private:
    RosUiBridge *bridge_;
};
