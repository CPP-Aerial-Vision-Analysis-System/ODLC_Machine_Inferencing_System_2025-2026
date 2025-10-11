// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#ifndef GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__TRAITS_HPP_
#define GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "gps_mavros/srv/detail/get_gps_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace gps_mavros
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGPSData_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGPSData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGPSData_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace gps_mavros

namespace rosidl_generator_traits
{

[[deprecated("use gps_mavros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gps_mavros::srv::GetGPSData_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  gps_mavros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gps_mavros::srv::to_yaml() instead")]]
inline std::string to_yaml(const gps_mavros::srv::GetGPSData_Request & msg)
{
  return gps_mavros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gps_mavros::srv::GetGPSData_Request>()
{
  return "gps_mavros::srv::GetGPSData_Request";
}

template<>
inline const char * name<gps_mavros::srv::GetGPSData_Request>()
{
  return "gps_mavros/srv/GetGPSData_Request";
}

template<>
struct has_fixed_size<gps_mavros::srv::GetGPSData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gps_mavros::srv::GetGPSData_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gps_mavros::srv::GetGPSData_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace gps_mavros
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetGPSData_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetGPSData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetGPSData_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace gps_mavros

namespace rosidl_generator_traits
{

[[deprecated("use gps_mavros::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const gps_mavros::srv::GetGPSData_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  gps_mavros::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use gps_mavros::srv::to_yaml() instead")]]
inline std::string to_yaml(const gps_mavros::srv::GetGPSData_Response & msg)
{
  return gps_mavros::srv::to_yaml(msg);
}

template<>
inline const char * data_type<gps_mavros::srv::GetGPSData_Response>()
{
  return "gps_mavros::srv::GetGPSData_Response";
}

template<>
inline const char * name<gps_mavros::srv::GetGPSData_Response>()
{
  return "gps_mavros/srv/GetGPSData_Response";
}

template<>
struct has_fixed_size<gps_mavros::srv::GetGPSData_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<gps_mavros::srv::GetGPSData_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<gps_mavros::srv::GetGPSData_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<gps_mavros::srv::GetGPSData>()
{
  return "gps_mavros::srv::GetGPSData";
}

template<>
inline const char * name<gps_mavros::srv::GetGPSData>()
{
  return "gps_mavros/srv/GetGPSData";
}

template<>
struct has_fixed_size<gps_mavros::srv::GetGPSData>
  : std::integral_constant<
    bool,
    has_fixed_size<gps_mavros::srv::GetGPSData_Request>::value &&
    has_fixed_size<gps_mavros::srv::GetGPSData_Response>::value
  >
{
};

template<>
struct has_bounded_size<gps_mavros::srv::GetGPSData>
  : std::integral_constant<
    bool,
    has_bounded_size<gps_mavros::srv::GetGPSData_Request>::value &&
    has_bounded_size<gps_mavros::srv::GetGPSData_Response>::value
  >
{
};

template<>
struct is_service<gps_mavros::srv::GetGPSData>
  : std::true_type
{
};

template<>
struct is_service_request<gps_mavros::srv::GetGPSData_Request>
  : std::true_type
{
};

template<>
struct is_service_response<gps_mavros::srv::GetGPSData_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__TRAITS_HPP_
