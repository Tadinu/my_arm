#ifndef HANDS_LISTENER_H
#define HANDS_LISTENER_H

#include <iostream>
#include <string.h>
#include "Leap.h"
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>

using namespace Leap;
using namespace std;

class HandsListener : public Listener {
  public:
  ros::NodeHandle _node;
  ros::Publisher _pub_marker_array;
  ros::Publisher _pub_bone_only;
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

#endif // HANDS_LISTENER_H
