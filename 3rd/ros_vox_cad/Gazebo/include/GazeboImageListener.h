#ifndef GAZEBO_IMAGE_LISTENER_H
#define GAZEBO_IMAGE_LISTENER_H

#include <QThread>
#include <QPainter>
#include <QLabel>
#include <QPixmap>
#include <QMutex>

#include <iostream>
#include <sstream>
#include <iomanip>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

class QLabel;
class QPixmap;

class GazeboImageListener : public QThread
{
    Q_OBJECT
	
public:
    typedef const boost::shared_ptr<gazebo::msgs::Image const> ConstImagePtr;
    void callback(ConstImagePtr &_msg);
    GazeboImageListener(int , char **,QMutex &,gazebo::common::Image&);
    ~GazeboImageListener();
protected:
    void run();

signals:
    void newImageArrived();

private:
    QLabel *label;
    QPixmap *pixmap;
    int argc;
    char **argv;
    QMutex &gazeboImageMutex;
    gazebo::common::Image &gazeboImage;
};

#endif
