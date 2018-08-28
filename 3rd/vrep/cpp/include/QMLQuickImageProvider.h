// =================================================================================================
#ifndef QML_QUICKIMAGE_PROVIDER_H
#define QML_QUICKIMAGE_PROVIDER_H

#include <qqmlextensionplugin.h>

#include <qqmlengine.h>
#include <qquickimageprovider.h>
#include <QImage>
#include <QPainter>

//![0]
class QMLQuickImageProvider : public QQuickImageProvider
{
    // Q_OBJECT (QQuickImageProvider is not a Q_OBJECT)
public:
    QMLQuickImageProvider();
    QMLQuickImageProvider(int sourceType);
    ~QMLQuickImageProvider();
    
    enum ImageSourceType
    {
        FRONT_VISION_SENSOR_IMAGE,
        GROUND_VISION_SENSOR_IMAGE
    };
    static constexpr const char* frontVisionSensorImageName = "frontVisionSensorImage";
    static constexpr const char* groundVisionSensorImageName = "groundVisionSensorImage";

    int _sourceType;
    void updateFromVisionSensor(const QString& prefix);
    QString getImageId(int index);

    QImage requestImage(const QString &id, QSize *size, const QSize& requestedSize);
    QPixmap requestPixmap(const QString &id, QSize *size, const QSize &requestedSize);
    QQmlImageProviderBase::ImageType imageType() const;
    
protected:
    void clear();
    void addImageId(const QString& prefix, int index);
    void remImageId(int index);

private:
    QMap<QString, int> _imageIdMap; // <objImageId, index>
    QVector<QString> _imageIdList;
};
//![1]


// ================================================================================================

#endif
