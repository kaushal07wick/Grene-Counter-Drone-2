// Generated by gencpp from file controller_msgs/FlatTarget.msg
// DO NOT EDIT!


#ifndef CONTROLLER_MSGS_MESSAGE_FLATTARGET_H
#define CONTROLLER_MSGS_MESSAGE_FLATTARGET_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace controller_msgs
{
template <class ContainerAllocator>
struct FlatTarget_
{
  typedef FlatTarget_<ContainerAllocator> Type;

  FlatTarget_()
    : header()
    , type_mask(0)
    , position()
    , velocity()
    , acceleration()
    , jerk()
    , snap()  {
    }
  FlatTarget_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , type_mask(0)
    , position(_alloc)
    , velocity(_alloc)
    , acceleration(_alloc)
    , jerk(_alloc)
    , snap(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _type_mask_type;
  _type_mask_type type_mask;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _acceleration_type;
  _acceleration_type acceleration;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _jerk_type;
  _jerk_type jerk;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _snap_type;
  _snap_type snap;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(IGNORE_SNAP)
  #undef IGNORE_SNAP
#endif
#if defined(_WIN32) && defined(IGNORE_SNAP_JERK)
  #undef IGNORE_SNAP_JERK
#endif
#if defined(_WIN32) && defined(IGNORE_SNAP_JERK_ACC)
  #undef IGNORE_SNAP_JERK_ACC
#endif

  enum {
    IGNORE_SNAP = 1u,
    IGNORE_SNAP_JERK = 2u,
    IGNORE_SNAP_JERK_ACC = 4u,
  };


  typedef boost::shared_ptr< ::controller_msgs::FlatTarget_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::controller_msgs::FlatTarget_<ContainerAllocator> const> ConstPtr;

}; // struct FlatTarget_

typedef ::controller_msgs::FlatTarget_<std::allocator<void> > FlatTarget;

typedef boost::shared_ptr< ::controller_msgs::FlatTarget > FlatTargetPtr;
typedef boost::shared_ptr< ::controller_msgs::FlatTarget const> FlatTargetConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::controller_msgs::FlatTarget_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::controller_msgs::FlatTarget_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::controller_msgs::FlatTarget_<ContainerAllocator1> & lhs, const ::controller_msgs::FlatTarget_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.type_mask == rhs.type_mask &&
    lhs.position == rhs.position &&
    lhs.velocity == rhs.velocity &&
    lhs.acceleration == rhs.acceleration &&
    lhs.jerk == rhs.jerk &&
    lhs.snap == rhs.snap;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::controller_msgs::FlatTarget_<ContainerAllocator1> & lhs, const ::controller_msgs::FlatTarget_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace controller_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::controller_msgs::FlatTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::controller_msgs::FlatTarget_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::controller_msgs::FlatTarget_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::controller_msgs::FlatTarget_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller_msgs::FlatTarget_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::controller_msgs::FlatTarget_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::controller_msgs::FlatTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c62e72c40204947fb0e6a39a53afefad";
  }

  static const char* value(const ::controller_msgs::FlatTarget_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc62e72c40204947fULL;
  static const uint64_t static_value2 = 0xb0e6a39a53afefadULL;
};

template<class ContainerAllocator>
struct DataType< ::controller_msgs::FlatTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "controller_msgs/FlatTarget";
  }

  static const char* value(const ::controller_msgs::FlatTarget_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::controller_msgs::FlatTarget_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# reference for polynomial trajectory tracking\n"
"#\n"
"\n"
"std_msgs/Header header\n"
"\n"
"uint8 type_mask\n"
"uint8 IGNORE_SNAP = 1	# Position Velocity Acceleration Jerk Reference\n"
"uint8 IGNORE_SNAP_JERK = 2	# Position Velocity Acceleration Reference\n"
"uint8 IGNORE_SNAP_JERK_ACC = 4	# Position Reference\n"
"\n"
"geometry_msgs/Vector3 position\n"
"geometry_msgs/Vector3 velocity\n"
"geometry_msgs/Vector3 acceleration\n"
"geometry_msgs/Vector3 jerk\n"
"geometry_msgs/Vector3 snap\n"
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
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::controller_msgs::FlatTarget_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::controller_msgs::FlatTarget_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.type_mask);
      stream.next(m.position);
      stream.next(m.velocity);
      stream.next(m.acceleration);
      stream.next(m.jerk);
      stream.next(m.snap);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FlatTarget_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::controller_msgs::FlatTarget_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::controller_msgs::FlatTarget_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "type_mask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.type_mask);
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
    s << indent << "acceleration: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.acceleration);
    s << indent << "jerk: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.jerk);
    s << indent << "snap: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.snap);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CONTROLLER_MSGS_MESSAGE_FLATTARGET_H
