// Generated by gencpp from file kinova_msgs/StartResponse.msg
// DO NOT EDIT!


#ifndef KINOVA_MSGS_MESSAGE_STARTRESPONSE_H
#define KINOVA_MSGS_MESSAGE_STARTRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace kinova_msgs
{
template <class ContainerAllocator>
struct StartResponse_
{
  typedef StartResponse_<ContainerAllocator> Type;

  StartResponse_()
    : start_result()  {
    }
  StartResponse_(const ContainerAllocator& _alloc)
    : start_result(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _start_result_type;
  _start_result_type start_result;





  typedef boost::shared_ptr< ::kinova_msgs::StartResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::kinova_msgs::StartResponse_<ContainerAllocator> const> ConstPtr;

}; // struct StartResponse_

typedef ::kinova_msgs::StartResponse_<std::allocator<void> > StartResponse;

typedef boost::shared_ptr< ::kinova_msgs::StartResponse > StartResponsePtr;
typedef boost::shared_ptr< ::kinova_msgs::StartResponse const> StartResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::kinova_msgs::StartResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::kinova_msgs::StartResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace kinova_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsMessage': True, 'HasHeader': False, 'IsFixedSize': False}
// {'kinova_msgs': ['/home/hu/ProjectOtter/pararl_2/ROS/src/kinova-ros/kinova_msgs/msg', '/home/hu/ProjectOtter/pararl_2/ROS/devel/share/kinova_msgs/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::StartResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::kinova_msgs::StartResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::StartResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::kinova_msgs::StartResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::StartResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::kinova_msgs::StartResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::kinova_msgs::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e762e31d813526eaaa6a12e8354174fc";
  }

  static const char* value(const ::kinova_msgs::StartResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe762e31d813526eaULL;
  static const uint64_t static_value2 = 0xaa6a12e8354174fcULL;
};

template<class ContainerAllocator>
struct DataType< ::kinova_msgs::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "kinova_msgs/StartResponse";
  }

  static const char* value(const ::kinova_msgs::StartResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::kinova_msgs::StartResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string start_result\n\
\n\
";
  }

  static const char* value(const ::kinova_msgs::StartResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::kinova_msgs::StartResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.start_result);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StartResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::kinova_msgs::StartResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::kinova_msgs::StartResponse_<ContainerAllocator>& v)
  {
    s << indent << "start_result: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.start_result);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KINOVA_MSGS_MESSAGE_STARTRESPONSE_H
