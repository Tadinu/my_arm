#include "GazeboImageListener.h"

GazeboImageListener::GazeboImageListener(int _argc, char **_argv,QMutex &m, gazebo::common::Image &img):
                     gazeboImageMutex(m), gazeboImage(img)
{
    argc=_argc;
    argv=_argv;
    std::cout<<"In Constructor "<<argc<<" "<<argv[0]<<std::endl;
}

GazeboImageListener::~GazeboImageListener()
{
    gazebo::transport::fini();
}

void GazeboImageListener::callback(ConstImagePtr &_msg)
{
    std::string img_string=_msg->data();
    unsigned char *_image=(unsigned char *)img_string.c_str();
    unsigned int _width=_msg->width();
    unsigned int _height=_msg->height();

gazeboImageMutex.lock();
    gazeboImage.SetFromData(_image,_width,_height, gazebo::common::Image::RGBA_INT8);
gazeboImageMutex.unlock();

    emit newImageArrived();
}


void GazeboImageListener::run()
{
    std::cout<<"In Thread"<<std::endl;
    //gazebo::load(_argc, _argv);
    gazebo::transport::init();
    gazebo::transport::run();

    //Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/gazebo_camera_streaming", &GazeboImageListener::callback,this);

    // Busy wait loop...replace with your own code as needed.
    while(1);
    std::cout<<"Exit thread::run\n"<<std::endl;
}
