// Generated by gencpp from file traversability_msgs/FootprintPath.msg
// DO NOT EDIT!


#ifndef TRAVERSABILITY_MSGS_MESSAGE_FOOTPRINTPATH_H
#define TRAVERSABILITY_MSGS_MESSAGE_FOOTPRINTPATH_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PolygonStamped.h>

namespace traversability_msgs
{
template <class ContainerAllocator>
struct FootprintPath_
{
  typedef FootprintPath_<ContainerAllocator> Type;

  FootprintPath_()
    : poses()
    , radius(0.0)
    , footprint()
    , conservative(false)
    , compute_untraversable_polygon(false)  {
    }
  FootprintPath_(const ContainerAllocator& _alloc)
    : poses(_alloc)
    , radius(0.0)
    , footprint(_alloc)
    , conservative(false)
    , compute_untraversable_polygon(false)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::PoseArray_<ContainerAllocator>  _poses_type;
  _poses_type poses;

   typedef double _radius_type;
  _radius_type radius;

   typedef  ::geometry_msgs::PolygonStamped_<ContainerAllocator>  _footprint_type;
  _footprint_type footprint;

   typedef uint8_t _conservative_type;
  _conservative_type conservative;

   typedef uint8_t _compute_untraversable_polygon_type;
  _compute_untraversable_polygon_type compute_untraversable_polygon;





  typedef boost::shared_ptr< ::traversability_msgs::FootprintPath_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traversability_msgs::FootprintPath_<ContainerAllocator> const> ConstPtr;

}; // struct FootprintPath_

typedef ::traversability_msgs::FootprintPath_<std::allocator<void> > FootprintPath;

typedef boost::shared_ptr< ::traversability_msgs::FootprintPath > FootprintPathPtr;
typedef boost::shared_ptr< ::traversability_msgs::FootprintPath const> FootprintPathConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traversability_msgs::FootprintPath_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traversability_msgs::FootprintPath_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::traversability_msgs::FootprintPath_<ContainerAllocator1> & lhs, const ::traversability_msgs::FootprintPath_<ContainerAllocator2> & rhs)
{
  return lhs.poses == rhs.poses &&
    lhs.radius == rhs.radius &&
    lhs.footprint == rhs.footprint &&
    lhs.conservative == rhs.conservative &&
    lhs.compute_untraversable_polygon == rhs.compute_untraversable_polygon;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::traversability_msgs::FootprintPath_<ContainerAllocator1> & lhs, const ::traversability_msgs::FootprintPath_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace traversability_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::FootprintPath_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::FootprintPath_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::FootprintPath_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "6a088c23f98a81d850097dff6ef7145d";
  }

  static const char* value(const ::traversability_msgs::FootprintPath_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x6a088c23f98a81d8ULL;
  static const uint64_t static_value2 = 0x50097dff6ef7145dULL;
};

template<class ContainerAllocator>
struct DataType< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traversability_msgs/FootprintPath";
  }

  static const char* value(const ::traversability_msgs::FootprintPath_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Path of the footprint defined as array of connected poses.\n"
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

  static const char* value(const ::traversability_msgs::FootprintPath_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.poses);
      stream.next(m.radius);
      stream.next(m.footprint);
      stream.next(m.conservative);
      stream.next(m.compute_untraversable_polygon);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FootprintPath_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traversability_msgs::FootprintPath_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::traversability_msgs::FootprintPath_<ContainerAllocator>& v)
  {
    s << indent << "poses: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseArray_<ContainerAllocator> >::stream(s, indent + "  ", v.poses);
    s << indent << "radius: ";
    Printer<double>::stream(s, indent + "  ", v.radius);
    s << indent << "footprint: ";
    s << std::endl;
    Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.footprint);
    s << indent << "conservative: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.conservative);
    s << indent << "compute_untraversable_polygon: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.compute_untraversable_polygon);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TRAVERSABILITY_MSGS_MESSAGE_FOOTPRINTPATH_H
