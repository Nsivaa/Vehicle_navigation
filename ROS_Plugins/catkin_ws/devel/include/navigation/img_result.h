// Generated by gencpp from file navigation/img_result.msg
// DO NOT EDIT!


#ifndef NAVIGATION_MESSAGE_IMG_RESULT_H
#define NAVIGATION_MESSAGE_IMG_RESULT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace navigation
{
template <class ContainerAllocator>
struct img_result_
{
  typedef img_result_<ContainerAllocator> Type;

  img_result_()
    : red()
    , shift(0)  {
    }
  img_result_(const ContainerAllocator& _alloc)
    : red(_alloc)
    , shift(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _red_type;
  _red_type red;

   typedef int16_t _shift_type;
  _shift_type shift;





  typedef boost::shared_ptr< ::navigation::img_result_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::navigation::img_result_<ContainerAllocator> const> ConstPtr;

}; // struct img_result_

typedef ::navigation::img_result_<std::allocator<void> > img_result;

typedef boost::shared_ptr< ::navigation::img_result > img_resultPtr;
typedef boost::shared_ptr< ::navigation::img_result const> img_resultConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::navigation::img_result_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::navigation::img_result_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::navigation::img_result_<ContainerAllocator1> & lhs, const ::navigation::img_result_<ContainerAllocator2> & rhs)
{
  return lhs.red == rhs.red &&
    lhs.shift == rhs.shift;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::navigation::img_result_<ContainerAllocator1> & lhs, const ::navigation::img_result_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace navigation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::navigation::img_result_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::navigation::img_result_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation::img_result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::navigation::img_result_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::img_result_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::navigation::img_result_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::navigation::img_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d21b5d7806ece2676a2ff5c007553324";
  }

  static const char* value(const ::navigation::img_result_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd21b5d7806ece267ULL;
  static const uint64_t static_value2 = 0x6a2ff5c007553324ULL;
};

template<class ContainerAllocator>
struct DataType< ::navigation::img_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "navigation/img_result";
  }

  static const char* value(const ::navigation::img_result_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::navigation::img_result_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string red \n"
"int16 shift\n"
"\n"
;
  }

  static const char* value(const ::navigation::img_result_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::navigation::img_result_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.red);
      stream.next(m.shift);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct img_result_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::navigation::img_result_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::navigation::img_result_<ContainerAllocator>& v)
  {
    s << indent << "red: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.red);
    s << indent << "shift: ";
    Printer<int16_t>::stream(s, indent + "  ", v.shift);
  }
};

} // namespace message_operations
} // namespace ros

#endif // NAVIGATION_MESSAGE_IMG_RESULT_H