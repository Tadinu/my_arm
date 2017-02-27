#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/rendering/Camera.hh>
#include <string>
#include <iostream>
#include <sstream>

typedef const boost::shared_ptr<gazebo::msgs::Image const> ConstImagePtr;

#define COUT_PREFIX "\033[1;33m" << "[GazeboCameraPublisher] " << "\033[0m"
using namespace gazebo;
using namespace std;

class GazeboCameraPublisher: public CameraPlugin
{
public:virtual ~GazeboCameraPublisher()
{
	cout<<COUT_PREFIX<<"in destructor"<<endl;
}

public:void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{

	node = transport::NodePtr(new transport::Node());
	node->Init("default");
    publisher =node->Advertise<gazebo::msgs::Image>("~/gazebo_camera_streaming");
	CameraPlugin::Load(_parent, _sdf);
	this->parentSensor->SetActive(true);
}

public: void OnNewFrame(const unsigned char *_image,  unsigned int _width, unsigned int _height, unsigned int _depth, const std::string &_format)
{
	// Create a  image message
    msgs::Image cameraMsg;
	_image_ptr=_image;
    cameraMsg.set_width(_width);
    cameraMsg.set_height(_height);
    cameraMsg.set_pixel_format(gazebo::common::Image::RGB_INT8);
    cameraMsg.set_data(_image_ptr, _width*_height*3);
    cameraMsg.set_step(0);
    publisher->Publish(cameraMsg);
}

const unsigned char *_image_ptr;
gazebo::transport::NodePtr node;
gazebo::transport::PublisherPtr publisher;

};

GZ_REGISTER_SENSOR_PLUGIN(GazeboCameraPublisher)

