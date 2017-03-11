// Generated by gencpp from file obstacle_detector/car_destination.msg
// DO NOT EDIT!


#ifndef OBSTACLE_DETECTOR_MESSAGE_CAR_DESTINATION_H
#define OBSTACLE_DETECTOR_MESSAGE_CAR_DESTINATION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace obstacle_detector
{
template <class ContainerAllocator>
struct car_destination_
{
  typedef car_destination_<ContainerAllocator> Type;

  car_destination_()
    : header()
    , pose()  {
    }
  car_destination_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;




  typedef boost::shared_ptr< ::obstacle_detector::car_destination_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::obstacle_detector::car_destination_<ContainerAllocator> const> ConstPtr;

}; // struct car_destination_

typedef ::obstacle_detector::car_destination_<std::allocator<void> > car_destination;

typedef boost::shared_ptr< ::obstacle_detector::car_destination > car_destinationPtr;
typedef boost::shared_ptr< ::obstacle_detector::car_destination const> car_destinationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::obstacle_detector::car_destination_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::obstacle_detector::car_destination_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace obstacle_detector

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'obstacle_detector': ['/home/dshin/fydp/adas-car-on-treadmill-v2/src/obstacle_detector/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::car_destination_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::obstacle_detector::car_destination_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::car_destination_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::obstacle_detector::car_destination_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::car_destination_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::obstacle_detector::car_destination_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::obstacle_detector::car_destination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d3812c3cbc69362b77dc0b19b345f8f5";
  }

  static const char* value(const ::obstacle_detector::car_destination_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd3812c3cbc69362bULL;
  static const uint64_t static_value2 = 0x77dc0b19b345f8f5ULL;
};

template<class ContainerAllocator>
struct DataType< ::obstacle_detector::car_destination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "obstacle_detector/car_destination";
  }

  static const char* value(const ::obstacle_detector::car_destination_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::obstacle_detector::car_destination_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header header\n\
geometry_msgs/Pose pose\n\
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
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
";
  }

  static const char* value(const ::obstacle_detector::car_destination_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::obstacle_detector::car_destination_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct car_destination_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::obstacle_detector::car_destination_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::obstacle_detector::car_destination_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OBSTACLE_DETECTOR_MESSAGE_CAR_DESTINATION_H
