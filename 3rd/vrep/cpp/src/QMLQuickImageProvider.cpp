#include "QMLQuickImageProvider.h"
#include "RbRobotSensorAdapter.h"

QMLQuickImageProvider::QMLQuickImageProvider()
                      : QQuickImageProvider(QQuickImageProvider::Image),
                      _sourceType(ImageSourceType::FRONT_VISION_SENSOR_IMAGE)
{
}

QMLQuickImageProvider::QMLQuickImageProvider(int sourceType)
                      : QQuickImageProvider(QQuickImageProvider::Image),
                        _sourceType(sourceType)
{
}

QQmlImageProviderBase::ImageType QMLQuickImageProvider::imageType() const
{
    return QQmlImageProviderBase::Image;
}

QMLQuickImageProvider::~QMLQuickImageProvider()
{

}

/**********************************************************************************************
 * @fn  QImage QMLQuickImageProvider::requestImage(const QString &imageId, QSize *size,
 *      const QSize& )
 *
 * @brief   Request image.
 *
 * @author  Duc Than
 * @date    7/11/2016
 *          8/28/2018
 * @param   imageId         Identifier for the image.
 * @param [in,out]  size    If non-null, the size.
 * @param   parameter3      The third parameter.
 *
 * @return  A QImage.
 **************************************************************************************************/

QImage QMLQuickImageProvider::requestImage(const QString &imageId, QSize *, const QSize& /*requestedSize*/)
{
    switch (_sourceType) {
    case ImageSourceType::FRONT_VISION_SENSOR_IMAGE:
        return RB_SENSOR_SYSTEM()->getVisionSensorImage(RbRobotSensorAdapter::RB_SENSOR_FRONT_VISION);

    case ImageSourceType::GROUND_VISION_SENSOR_IMAGE:
        return RB_SENSOR_SYSTEM()->getVisionSensorImage(RbRobotSensorAdapter::RB_SENSOR_GROUND_VISION);
    }
    return QImage();
    /*

    if (size)
    *size = QSize(QMLAdapter::OBJECT_IMAGE_WIDTH, QMLAdapter::OBJECT_IMAGE_HEIGHT);

    QImage image;
    image.scaled(requestedSize.width() > 0 ? requestedSize.width() : width,
        requestedSize.height() > 0 ? requestedSize.height() : height);

    return image;
    */
}

QPixmap QMLQuickImageProvider::requestPixmap(const QString &imageId, QSize *size, const QSize & requestedSize)
{
    return QPixmap::fromImage(this->requestImage(imageId, size, requestedSize));
}

void QMLQuickImageProvider::clear()
{
    _imageIdMap.clear();
    _imageIdList.clear();
}

QString QMLQuickImageProvider::getImageId(int index)
{
    if (index >= 0 && index < _imageIdList.size())
        return _imageIdList[index]; // ALSO _imageIdMap.key(index)

    //assert(0);
    return "";
}

void QMLQuickImageProvider::addImageId(const QString& prefix, int index)
{
    //assert(index == _imageIdList.size());
    const QString imgName = QString(prefix+"%1").arg(index);
    _imageIdMap.insert(imgName, index);
    _imageIdList.append(imgName);
}

void QMLQuickImageProvider::remImageId(int index)
{
    assert(index < _imageIdList.size());
    _imageIdList.remove(index);
    /*for (int i = index; i < _imageIdList.size(); ++i)
        _imageIdList[i] = QString("obj%1").arg(i);*/

    QMap<QString, int> tempMap;
    for (int i = 0; i < _imageIdList.size(); ++i)
        tempMap.insert(_imageIdList[i], i);

    _imageIdMap.swap(tempMap);
}

void QMLQuickImageProvider::updateFromFrontVisionSensor()
{
    clear();
    this->addImageId("front", 0);
}

void QMLQuickImageProvider::updateFromFloorVisionSensor()
{
    clear();
    this->addImageId("floor", 0);
}
