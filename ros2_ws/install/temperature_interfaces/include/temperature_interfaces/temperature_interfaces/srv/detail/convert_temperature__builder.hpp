// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
// generated code does not contain a copyright notice

#ifndef TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__BUILDER_HPP_
#define TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "temperature_interfaces/srv/detail/convert_temperature__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace temperature_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConvertTemperature_Request_conversion_type
{
public:
  explicit Init_ConvertTemperature_Request_conversion_type(::temperature_interfaces::srv::ConvertTemperature_Request & msg)
  : msg_(msg)
  {}
  ::temperature_interfaces::srv::ConvertTemperature_Request conversion_type(::temperature_interfaces::srv::ConvertTemperature_Request::_conversion_type_type arg)
  {
    msg_.conversion_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::temperature_interfaces::srv::ConvertTemperature_Request msg_;
};

class Init_ConvertTemperature_Request_input_temp
{
public:
  Init_ConvertTemperature_Request_input_temp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ConvertTemperature_Request_conversion_type input_temp(::temperature_interfaces::srv::ConvertTemperature_Request::_input_temp_type arg)
  {
    msg_.input_temp = std::move(arg);
    return Init_ConvertTemperature_Request_conversion_type(msg_);
  }

private:
  ::temperature_interfaces::srv::ConvertTemperature_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::temperature_interfaces::srv::ConvertTemperature_Request>()
{
  return temperature_interfaces::srv::builder::Init_ConvertTemperature_Request_input_temp();
}

}  // namespace temperature_interfaces


namespace temperature_interfaces
{

namespace srv
{

namespace builder
{

class Init_ConvertTemperature_Response_converted_temp
{
public:
  Init_ConvertTemperature_Response_converted_temp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::temperature_interfaces::srv::ConvertTemperature_Response converted_temp(::temperature_interfaces::srv::ConvertTemperature_Response::_converted_temp_type arg)
  {
    msg_.converted_temp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::temperature_interfaces::srv::ConvertTemperature_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::temperature_interfaces::srv::ConvertTemperature_Response>()
{
  return temperature_interfaces::srv::builder::Init_ConvertTemperature_Response_converted_temp();
}

}  // namespace temperature_interfaces

#endif  // TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__BUILDER_HPP_
