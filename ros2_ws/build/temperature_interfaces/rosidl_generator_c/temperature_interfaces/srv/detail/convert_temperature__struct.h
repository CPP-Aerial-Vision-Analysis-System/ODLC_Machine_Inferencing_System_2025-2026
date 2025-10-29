// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
// generated code does not contain a copyright notice

#ifndef TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__STRUCT_H_
#define TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'conversion_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ConvertTemperature in the package temperature_interfaces.
typedef struct temperature_interfaces__srv__ConvertTemperature_Request
{
  double input_temp;
  /// "FtoC" or "CtoF"
  rosidl_runtime_c__String conversion_type;
} temperature_interfaces__srv__ConvertTemperature_Request;

// Struct for a sequence of temperature_interfaces__srv__ConvertTemperature_Request.
typedef struct temperature_interfaces__srv__ConvertTemperature_Request__Sequence
{
  temperature_interfaces__srv__ConvertTemperature_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} temperature_interfaces__srv__ConvertTemperature_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ConvertTemperature in the package temperature_interfaces.
typedef struct temperature_interfaces__srv__ConvertTemperature_Response
{
  double converted_temp;
} temperature_interfaces__srv__ConvertTemperature_Response;

// Struct for a sequence of temperature_interfaces__srv__ConvertTemperature_Response.
typedef struct temperature_interfaces__srv__ConvertTemperature_Response__Sequence
{
  temperature_interfaces__srv__ConvertTemperature_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} temperature_interfaces__srv__ConvertTemperature_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEMPERATURE_INTERFACES__SRV__DETAIL__CONVERT_TEMPERATURE__STRUCT_H_
