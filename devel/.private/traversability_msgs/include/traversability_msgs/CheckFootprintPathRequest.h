// Generated by gencpp from file traversability_msgs/CheckFootprintPathRequest.msg
// DO NOT EDIT!


#ifndef TRAVERSABILITY_MSGS_MESSAGE_CHECKFOOTPRINTPATHREQUEST_H
#define TRAVERSABILITY_MSGS_MESSAGE_CHECKFOOTPRINTPATHREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <traversability_msgs/FootprintPath.h>

namespace traversability_msgs
{
template <class ContainerAllocator>
struct CheckFootprintPathRequest_
{
  typedef CheckFootprintPathRequest_<ContainerAllocator> Type;

  CheckFootprintPathRequest_()
    : path()  {
    }
  CheckFootprintPathRequest_(const ContainerAllocator& _alloc)
    : path(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector< ::traversability_msgs::FootprintPath_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::traversability_msgs::FootprintPath_<ContainerAllocator> >> _path_type;
  _path_type path;





  typedef boost::shared_ptr< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> const> ConstPtr;

}; // struct CheckFootprintPathRequest_

typedef ::traversability_msgs::CheckFootprintPathRequest_<std::allocator<void> > CheckFootprintPathRequest;

typedef boost::shared_ptr< ::traversability_msgs::CheckFootprintPathRequest > CheckFootprintPathRequestPtr;
typedef boost::shared_ptr< ::traversability_msgs::CheckFootprintPathRequest const> CheckFootprintPathRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator1> & lhs, const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator2> & rhs)
{
  return lhs.path == rhs.path;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator1> & lhs, const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace traversability_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "715ac8adb4922a1589b635a04ab24fb2";
  }

  static const char* value(const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x715ac8adb4922a15ULL;
  static const uint64_t static_value2 = 0x89b635a04ab24fb2ULL;
};

template<class ContainerAllocator>
struct DataType< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traversability_msgs/CheckFootprintPathRequest";
  }

  static const char* value(const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Footprint path to check.\n"
"traversability_msgs/FootprintPath[] path\n"
"\n"
"\n"
"================================================================================\n"
"MSG: traversability_msgs/FootprintPath\n"
"# Path of the footprint defined as array of connected poses.\n"
"# Poses are connected piece-wise linear in the order of the array.\n"
"geometry_msgs/PoseArray poses\n"
"\n"
"# Either: Define footprint radius.\n"
"float64 radius \n"
"\n"
"# Or: Define footprint as polygon in the robot base frame.\n"
"# Polygon is used if it is defined, otherwise radius is used.\n"
"geometry_msgs/PolygonStamped footprint\n"
"\n"
"# Use conservative footprint. Only available if polygon is used.\n"
"bool conservative\n"
"\n"
"# Compute untraversable polygon in the checked area for traversability. If true, computation demand is higher.\n"
"bool compute_untraversable_polygon\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseArray\n"
"# An array of poses with a header for global reference.\n"
"\n"
"Header header\n"
"\n"
"Pose[] poses\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PolygonStamped\n"
"# This represents a Polygon with reference coordinate frame and timestamp\n"
"Header header\n"
"Polygon polygon\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Polygon\n"
"#A specification of a polygon where the first and last points are assumed to be connected\n"
"Point32[] points\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point32\n"
"# This contains the position of a point in free space(with 32 bits of precision).\n"
"# It is recommeded to use Point wherever possible instead of Point32.  \n"
"# \n"
"# This recommendation is to promote interoperability.  \n"
"#\n"
"# This message is designed to take up less space when sending\n"
"# lots of points at once, as in the case of a PointCloud.  \n"
"\n"
"float32 x\n"
"float32 y\n"
"float32 z\n"
;
  }

  static const char* value(const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.path);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct CheckFootprintPathRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traversability_msgs::CheckFootprintPathRequest_<ContainerAllocator>& v)
  {
    s << indent << "path[]" << std::endl;
    for (size_t i = 0; i < v.path.size(); ++i)
    {
      s << indent << "  path[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::traversability_msgs::FootprintPath_<ContainerAllocator> >::stream(s, indent + "    ", v.path[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAVERSABILITY_MSGS_MESSAGE_CHECKFOOTPRINTPATHREQUEST_H
