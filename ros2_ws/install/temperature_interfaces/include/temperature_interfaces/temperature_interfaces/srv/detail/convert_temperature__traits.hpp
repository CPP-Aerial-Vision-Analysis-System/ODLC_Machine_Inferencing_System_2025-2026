// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
// generated code does not contain a copyright notice

#ifndef TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__TRAITS_HPP_
#define TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "temperature_interfaces/srv/detail/convert_temperature__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace temperature_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConvertTemperature_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: input_temp
  {
    out << "input_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.input_temp, out);
    out << ", ";
  }

  // member: conversion_type
  {
    out << "conversion_type: ";
    rosidl_generator_traits::value_to_yaml(msg.conversion_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConvertTemperature_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: input_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.input_temp, out);
    out << "\n";
  }

  // member: conversion_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "conversion_type: ";
    rosidl_generator_traits::value_to_yaml(msg.conversion_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConvertTemperature_Request & msg, bool use_flow_style = false)
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

}  // namespace temperature_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use temperature_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const temperature_interfaces::srv::ConvertTemperature_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  temperature_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use temperature_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const temperature_interfaces::srv::ConvertTemperature_Request & msg)
{
  return temperature_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<temperature_interfaces::srv::ConvertTemperature_Request>()
{
  return "temperature_interfaces::srv::ConvertTemperature_Request";
}

template<>
inline const char * name<temperature_interfaces::srv::ConvertTemperature_Request>()
{
  return "temperature_interfaces/srv/ConvertTemperature_Request";
}

template<>
struct has_fixed_size<temperature_interfaces::srv::ConvertTemperature_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<temperature_interfaces::srv::ConvertTemperature_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<temperature_interfaces::srv::ConvertTemperature_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace temperature_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ConvertTemperature_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: converted_temp
  {
    out << "converted_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.converted_temp, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ConvertTemperature_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: converted_temp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "converted_temp: ";
    rosidl_generator_traits::value_to_yaml(msg.converted_temp, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ConvertTemperature_Response & msg, bool use_flow_style = false)
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

}  // namespace temperature_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use temperature_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const temperature_interfaces::srv::ConvertTemperature_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  temperature_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use temperature_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const temperature_interfaces::srv::ConvertTemperature_Response & msg)
{
  return temperature_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<temperature_interfaces::srv::ConvertTemperature_Response>()
{
  return "temperature_interfaces::srv::ConvertTemperature_Response";
}

template<>
inline const char * name<temperature_interfaces::srv::ConvertTemperature_Response>()
{
  return "temperature_interfaces/srv/ConvertTemperature_Response";
}

template<>
struct has_fixed_size<temperature_interfaces::srv::ConvertTemperature_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<temperature_interfaces::srv::ConvertTemperature_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<temperature_interfaces::srv::ConvertTemperature_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<temperature_interfaces::srv::ConvertTemperature>()
{
  return "temperature_interfaces::srv::ConvertTemperature";
}

template<>
inline const char * name<temperature_interfaces::srv::ConvertTemperature>()
{
  return "temperature_interfaces/srv/ConvertTemperature";
}

template<>
struct has_fixed_size<temperature_interfaces::srv::ConvertTemperature>
  : std::integral_constant<
    bool,
    has_fixed_size<temperature_interfaces::srv::ConvertTemperature_Request>::value &&
    has_fixed_size<temperature_interfaces::srv::ConvertTemperature_Response>::value
  >
{
};

template<>
struct has_bounded_size<temperature_interfaces::srv::ConvertTemperature>
  : std::integral_constant<
    bool,
    has_bounded_size<temperature_interfaces::srv::ConvertTemperature_Request>::value &&
    has_bounded_size<temperature_interfaces::srv::ConvertTemperature_Response>::value
  >
{
};

template<>
struct is_service<temperature_interfaces::srv::ConvertTemperature>
  : std::true_type
{
};

template<>
struct is_service_request<temperature_interfaces::srv::ConvertTemperature_Request>
  : std::true_type
{
};

template<>
struct is_service_response<temperature_interfaces::srv::ConvertTemperature_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__TRAITS_HPP_
