// Generated by gencpp from file estimation/IRMarker.msg
// DO NOT EDIT!


#ifndef ESTIMATION_MESSAGE_IRMARKER_H
#define ESTIMATION_MESSAGE_IRMARKER_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/String.h>
#include <std_msgs/String.h>

namespace estimation
{
template <class ContainerAllocator>
struct IRMarker_
{
  typedef IRMarker_<ContainerAllocator> Type;

  IRMarker_()
    : landmarkID()
    , markerID()
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
    }
  IRMarker_(const ContainerAllocator& _alloc)
    : landmarkID(_alloc)
    , markerID(_alloc)
    , x(0.0)
    , y(0.0)
    , z(0.0)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::String_<ContainerAllocator>  _landmarkID_type;
  _landmarkID_type landmarkID;

   typedef  ::std_msgs::String_<ContainerAllocator>  _markerID_type;
  _markerID_type markerID;

   typedef float _x_type;
  _x_type x;

   typedef float _y_type;
  _y_type y;

   typedef float _z_type;
  _z_type z;





  typedef boost::shared_ptr< ::estimation::IRMarker_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::estimation::IRMarker_<ContainerAllocator> const> ConstPtr;

}; // struct IRMarker_

typedef ::estimation::IRMarker_<std::allocator<void> > IRMarker;

typedef boost::shared_ptr< ::estimation::IRMarker > IRMarkerPtr;
typedef boost::shared_ptr< ::estimation::IRMarker const> IRMarkerConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::estimation::IRMarker_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::estimation::IRMarker_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace estimation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'estimation': ['/home/mavlab/develop/superstate/mpcROS/src/estimation/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::estimation::IRMarker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::estimation::IRMarker_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::estimation::IRMarker_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::estimation::IRMarker_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::estimation::IRMarker_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::estimation::IRMarker_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::estimation::IRMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "74186740d14e922a7e5ca083a6795f31";
  }

  static const char* value(const ::estimation::IRMarker_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x74186740d14e922aULL;
  static const uint64_t static_value2 = 0x7e5ca083a6795f31ULL;
};

template<class ContainerAllocator>
struct DataType< ::estimation::IRMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "estimation/IRMarker";
  }

  static const char* value(const ::estimation::IRMarker_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::estimation::IRMarker_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ROS Message that contains pixel-space IR marker locations and labels.\n\
\n\
# ID of the landmark (e.g. gate) that the IR marker is attached to\n\
std_msgs/String landmarkID\n\
# ID of individual marker.\n\
std_msgs/String markerID\n\
\n\
float32 x\n\
float32 y\n\
# Z is the distance from the camera in meters.\n\
float32 z\n\
================================================================================\n\
MSG: std_msgs/String\n\
string data\n\
";
  }

  static const char* value(const ::estimation::IRMarker_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::estimation::IRMarker_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.landmarkID);
      stream.next(m.markerID);
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct IRMarker_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::estimation::IRMarker_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::estimation::IRMarker_<ContainerAllocator>& v)
  {
    s << indent << "landmarkID: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.landmarkID);
    s << indent << "markerID: ";
    s << std::endl;
    Printer< ::std_msgs::String_<ContainerAllocator> >::stream(s, indent + "  ", v.markerID);
    s << indent << "x: ";
    Printer<float>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<float>::stream(s, indent + "  ", v.y);
    s << indent << "z: ";
    Printer<float>::stream(s, indent + "  ", v.z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // ESTIMATION_MESSAGE_IRMARKER_H
