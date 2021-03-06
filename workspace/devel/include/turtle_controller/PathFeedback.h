// Generated by gencpp from file turtle_controller/PathFeedback.msg
// DO NOT EDIT!


#ifndef TURTLE_CONTROLLER_MESSAGE_PATHFEEDBACK_H
#define TURTLE_CONTROLLER_MESSAGE_PATHFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace turtle_controller
{
template <class ContainerAllocator>
struct PathFeedback_
{
  typedef PathFeedback_<ContainerAllocator> Type;

  PathFeedback_()
    : progress(0.0)
    , currentPosition()  {
    }
  PathFeedback_(const ContainerAllocator& _alloc)
    : progress(0.0)
    , currentPosition(_alloc)  {
  (void)_alloc;
    }



   typedef float _progress_type;
  _progress_type progress;

   typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _currentPosition_type;
  _currentPosition_type currentPosition;




  typedef boost::shared_ptr< ::turtle_controller::PathFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::turtle_controller::PathFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct PathFeedback_

typedef ::turtle_controller::PathFeedback_<std::allocator<void> > PathFeedback;

typedef boost::shared_ptr< ::turtle_controller::PathFeedback > PathFeedbackPtr;
typedef boost::shared_ptr< ::turtle_controller::PathFeedback const> PathFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::turtle_controller::PathFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::turtle_controller::PathFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace turtle_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'turtle_controller': ['/home/agustin/Agustin/Work/Ekumen/Localwork/ros_turtle/workspace/devel/share/turtle_controller/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::turtle_controller::PathFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::turtle_controller::PathFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::turtle_controller::PathFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::turtle_controller::PathFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turtle_controller::PathFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::turtle_controller::PathFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::turtle_controller::PathFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dc3da8d59bf639e439e08c2db57db40f";
  }

  static const char* value(const ::turtle_controller::PathFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdc3da8d59bf639e4ULL;
  static const uint64_t static_value2 = 0x39e08c2db57db40fULL;
};

template<class ContainerAllocator>
struct DataType< ::turtle_controller::PathFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "turtle_controller/PathFeedback";
  }

  static const char* value(const ::turtle_controller::PathFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::turtle_controller::PathFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
float32 progress\n\
float32[] currentPosition\n\
";
  }

  static const char* value(const ::turtle_controller::PathFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::turtle_controller::PathFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.progress);
      stream.next(m.currentPosition);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PathFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::turtle_controller::PathFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::turtle_controller::PathFeedback_<ContainerAllocator>& v)
  {
    s << indent << "progress: ";
    Printer<float>::stream(s, indent + "  ", v.progress);
    s << indent << "currentPosition[]" << std::endl;
    for (size_t i = 0; i < v.currentPosition.size(); ++i)
    {
      s << indent << "  currentPosition[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.currentPosition[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TURTLE_CONTROLLER_MESSAGE_PATHFEEDBACK_H
