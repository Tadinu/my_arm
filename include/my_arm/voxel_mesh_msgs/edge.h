// Generated by gencpp from file voxel_mesh_msgs/edge.msg
// DO NOT EDIT!


#ifndef VOXEL_MESH_MSGS_MESSAGE_EDGE_H
#define VOXEL_MESH_MSGS_MESSAGE_EDGE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace voxel_mesh_msgs
{
template <class ContainerAllocator>
struct edge_
{
  typedef edge_<ContainerAllocator> Type;

  edge_()
    : header()
    , vi0(0)
    , vi1(0)  {
    }
  edge_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , vi0(0)
    , vi1(0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _vi0_type;
  _vi0_type vi0;

   typedef int32_t _vi1_type;
  _vi1_type vi1;




  typedef boost::shared_ptr< ::voxel_mesh_msgs::edge_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::voxel_mesh_msgs::edge_<ContainerAllocator> const> ConstPtr;

}; // struct edge_

typedef ::voxel_mesh_msgs::edge_<std::allocator<void> > edge;

typedef boost::shared_ptr< ::voxel_mesh_msgs::edge > edgePtr;
typedef boost::shared_ptr< ::voxel_mesh_msgs::edge const> edgeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::voxel_mesh_msgs::edge_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::voxel_mesh_msgs::edge_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace voxel_mesh_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'shape_msgs': ['/opt/ros/kinetic/share/shape_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'object_recognition_msgs': ['/opt/ros/kinetic/share/object_recognition_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'voxel_mesh_msgs': ['/home/brhm/DUC/RobotArm/src/voxel_mesh_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voxel_mesh_msgs::edge_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voxel_mesh_msgs::edge_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voxel_mesh_msgs::edge_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7177c2ec7d5f48268e24c27f25b12855";
  }

  static const char* value(const ::voxel_mesh_msgs::edge_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7177c2ec7d5f4826ULL;
  static const uint64_t static_value2 = 0x8e24c27f25b12855ULL;
};

template<class ContainerAllocator>
struct DataType< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
{
  static const char* value()
  {
    return "voxel_mesh_msgs/edge";
  }

  static const char* value(const ::voxel_mesh_msgs::edge_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
int32 vi0\n\
int32 vi1\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::voxel_mesh_msgs::edge_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.vi0);
      stream.next(m.vi1);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct edge_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::voxel_mesh_msgs::edge_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::voxel_mesh_msgs::edge_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "vi0: ";
    Printer<int32_t>::stream(s, indent + "  ", v.vi0);
    s << indent << "vi1: ";
    Printer<int32_t>::stream(s, indent + "  ", v.vi1);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VOXEL_MESH_MSGS_MESSAGE_EDGE_H