// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#ifndef GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_HPP_
#define GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__gps_mavros__srv__GetGPSData_Request __attribute__((deprecated))
#else
# define DEPRECATED__gps_mavros__srv__GetGPSData_Request __declspec(deprecated)
#endif

namespace gps_mavros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGPSData_Request_
{
  using Type = GetGPSData_Request_<ContainerAllocator>;

  explicit GetGPSData_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit GetGPSData_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gps_mavros__srv__GetGPSData_Request
    std::shared_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gps_mavros__srv__GetGPSData_Request
    std::shared_ptr<gps_mavros::srv::GetGPSData_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGPSData_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGPSData_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGPSData_Request_

// alias to use template instance with default allocator
using GetGPSData_Request =
  gps_mavros::srv::GetGPSData_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gps_mavros


#ifndef _WIN32
# define DEPRECATED__gps_mavros__srv__GetGPSData_Response __attribute__((deprecated))
#else
# define DEPRECATED__gps_mavros__srv__GetGPSData_Response __declspec(deprecated)
#endif

namespace gps_mavros
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetGPSData_Response_
{
  using Type = GetGPSData_Response_<ContainerAllocator>;

  explicit GetGPSData_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->yaw = 0.0;
    }
  }

  explicit GetGPSData_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->yaw = 0.0;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _yaw_type =
    double;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__yaw(
    const double & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__gps_mavros__srv__GetGPSData_Response
    std::shared_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__gps_mavros__srv__GetGPSData_Response
    std::shared_ptr<gps_mavros::srv::GetGPSData_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetGPSData_Response_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetGPSData_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetGPSData_Response_

// alias to use template instance with default allocator
using GetGPSData_Response =
  gps_mavros::srv::GetGPSData_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace gps_mavros

namespace gps_mavros
{

namespace srv
{

struct GetGPSData
{
  using Request = gps_mavros::srv::GetGPSData_Request;
  using Response = gps_mavros::srv::GetGPSData_Response;
};

}  // namespace srv

}  // namespace gps_mavros

#endif  // GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_HPP_
