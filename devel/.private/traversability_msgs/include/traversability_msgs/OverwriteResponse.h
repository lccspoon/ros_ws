// Generated by gencpp from file traversability_msgs/OverwriteResponse.msg
// DO NOT EDIT!


#ifndef TRAVERSABILITY_MSGS_MESSAGE_OVERWRITERESPONSE_H
#define TRAVERSABILITY_MSGS_MESSAGE_OVERWRITERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace traversability_msgs
{
template <class ContainerAllocator>
struct OverwriteResponse_
{
  typedef OverwriteResponse_<ContainerAllocator> Type;

  OverwriteResponse_()
    {
    }
  OverwriteResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> const> ConstPtr;

}; // struct OverwriteResponse_

typedef ::traversability_msgs::OverwriteResponse_<std::allocator<void> > OverwriteResponse;

typedef boost::shared_ptr< ::traversability_msgs::OverwriteResponse > OverwriteResponsePtr;
typedef boost::shared_ptr< ::traversability_msgs::OverwriteResponse const> OverwriteResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::traversability_msgs::OverwriteResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace traversability_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::traversability_msgs::OverwriteResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "traversability_msgs/OverwriteResponse";
  }

  static const char* value(const ::traversability_msgs::OverwriteResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::traversability_msgs::OverwriteResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OverwriteResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::traversability_msgs::OverwriteResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::traversability_msgs::OverwriteResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // TRAVERSABILITY_MSGS_MESSAGE_OVERWRITERESPONSE_H
