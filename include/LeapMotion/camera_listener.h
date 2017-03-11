#ifndef CAMERALISTENER_H
#define CAMERALISTENER_H

#include <iostream>
#include <string.h>
#include "Leap.h"
#include <ros/ros.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"

#include <boost/shared_ptr.hpp>
#include <sstream>

using namespace Leap;
using namespace std;

class CameraListener : public Listener {
  public:
  //ros::NodeHandle _node;
  boost::shared_ptr <ros::NodeHandle> _left_node;
  boost::shared_ptr <ros::NodeHandle> _right_node;

  ros::Publisher _pub_image_left;
  ros::Publisher _pub_info_left;
  ros::Publisher _pub_image_right;
  ros::Publisher _pub_info_right;
  camera_info_manager::CameraInfoManager* info_mgr_right;
  camera_info_manager::CameraInfoManager* info_mgr_left;
  unsigned int seq;
  virtual void onInit(const Controller&);
  virtual void onConnect(const Controller&);
  virtual void onDisconnect(const Controller&);
  virtual void onExit(const Controller&);
  virtual void onFrame(const Controller&);
  virtual void onFocusGained(const Controller&);
  virtual void onFocusLost(const Controller&);
  virtual void onDeviceChange(const Controller&);
  virtual void onServiceConnect(const Controller&);
  virtual void onServiceDisconnect(const Controller&);
  private:
};

#endif // CAMERALISTENER_H
