// Generated by gencpp from file opencvtest/pixel_point.msg
// DO NOT EDIT!


#ifndef OPENCVTEST_MESSAGE_PIXEL_POINT_H
#define OPENCVTEST_MESSAGE_PIXEL_POINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace opencvtest
{
template <class ContainerAllocator>
struct pixel_point_
{
  typedef pixel_point_<ContainerAllocator> Type;

  pixel_point_()
    : name()
    , u(0.0)
    , v(0.0)  {
    }
  pixel_point_(const ContainerAllocator& _alloc)
    : name(_alloc)
    , u(0.0)
    , v(0.0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _name_type;
  _name_type name;

   typedef float _u_type;
  _u_type u;

   typedef float _v_type;
  _v_type v;





  typedef boost::shared_ptr< ::opencvtest::pixel_point_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opencvtest::pixel_point_<ContainerAllocator> const> ConstPtr;

}; // struct pixel_point_

typedef ::opencvtest::pixel_point_<std::allocator<void> > pixel_point;

typedef boost::shared_ptr< ::opencvtest::pixel_point > pixel_pointPtr;
typedef boost::shared_ptr< ::opencvtest::pixel_point const> pixel_pointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opencvtest::pixel_point_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opencvtest::pixel_point_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace opencvtest

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'opencvtest': ['/home/zhangeaky/dobot_ws/src/opencvtest/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::opencvtest::pixel_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opencvtest::pixel_point_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencvtest::pixel_point_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencvtest::pixel_point_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencvtest::pixel_point_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencvtest::pixel_point_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opencvtest::pixel_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d73561c105114748b800885633d1c3c0";
  }

  static const char* value(const ::opencvtest::pixel_point_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd73561c105114748ULL;
  static const uint64_t static_value2 = 0xb800885633d1c3c0ULL;
};

template<class ContainerAllocator>
struct DataType< ::opencvtest::pixel_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opencvtest/pixel_point";
  }

  static const char* value(const ::opencvtest::pixel_point_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opencvtest::pixel_point_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string name\n\
float32 u\n\
float32 v\n\
\n\
";
  }

  static const char* value(const ::opencvtest::pixel_point_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opencvtest::pixel_point_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.name);
      stream.next(m.u);
      stream.next(m.v);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct pixel_point_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opencvtest::pixel_point_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opencvtest::pixel_point_<ContainerAllocator>& v)
  {
    s << indent << "name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.name);
    s << indent << "u: ";
    Printer<float>::stream(s, indent + "  ", v.u);
    s << indent << "v: ";
    Printer<float>::stream(s, indent + "  ", v.v);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENCVTEST_MESSAGE_PIXEL_POINT_H
