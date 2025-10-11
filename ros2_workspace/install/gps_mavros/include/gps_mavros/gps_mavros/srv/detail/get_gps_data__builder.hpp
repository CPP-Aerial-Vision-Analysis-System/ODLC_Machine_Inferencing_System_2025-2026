// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#ifndef GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__BUILDER_HPP_
#define GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gps_mavros/srv/detail/get_gps_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gps_mavros
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gps_mavros::srv::GetGPSData_Request>()
{
  return ::gps_mavros::srv::GetGPSData_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace gps_mavros


namespace gps_mavros
{

namespace srv
{

namespace builder
{

class Init_GetGPSData_Response_yaw
{
public:
  explicit Init_GetGPSData_Response_yaw(::gps_mavros::srv::GetGPSData_Response & msg)
  : msg_(msg)
  {}
  ::gps_mavros::srv::GetGPSData_Response yaw(::gps_mavros::srv::GetGPSData_Response::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gps_mavros::srv::GetGPSData_Response msg_;
};

class Init_GetGPSData_Response_altitude
{
public:
  explicit Init_GetGPSData_Response_altitude(::gps_mavros::srv::GetGPSData_Response & msg)
  : msg_(msg)
  {}
  Init_GetGPSData_Response_yaw altitude(::gps_mavros::srv::GetGPSData_Response::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GetGPSData_Response_yaw(msg_);
  }

private:
  ::gps_mavros::srv::GetGPSData_Response msg_;
};

class Init_GetGPSData_Response_longitude
{
public:
  explicit Init_GetGPSData_Response_longitude(::gps_mavros::srv::GetGPSData_Response & msg)
  : msg_(msg)
  {}
  Init_GetGPSData_Response_altitude longitude(::gps_mavros::srv::GetGPSData_Response::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GetGPSData_Response_altitude(msg_);
  }

private:
  ::gps_mavros::srv::GetGPSData_Response msg_;
};

class Init_GetGPSData_Response_latitude
{
public:
  Init_GetGPSData_Response_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGPSData_Response_longitude latitude(::gps_mavros::srv::GetGPSData_Response::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GetGPSData_Response_longitude(msg_);
  }

private:
  ::gps_mavros::srv::GetGPSData_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::gps_mavros::srv::GetGPSData_Response>()
{
  return gps_mavros::srv::builder::Init_GetGPSData_Response_latitude();
}

}  // namespace gps_mavros

#endif  // GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__BUILDER_HPP_
