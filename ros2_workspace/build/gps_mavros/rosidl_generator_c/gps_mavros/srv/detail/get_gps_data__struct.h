// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#ifndef GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_H_
#define GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetGPSData in the package gps_mavros.
typedef struct gps_mavros__srv__GetGPSData_Request
{
  uint8_t structure_needs_at_least_one_member;
} gps_mavros__srv__GetGPSData_Request;

// Struct for a sequence of gps_mavros__srv__GetGPSData_Request.
typedef struct gps_mavros__srv__GetGPSData_Request__Sequence
{
  gps_mavros__srv__GetGPSData_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gps_mavros__srv__GetGPSData_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetGPSData in the package gps_mavros.
typedef struct gps_mavros__srv__GetGPSData_Response
{
  double latitude;
  double longitude;
  double altitude;
  double yaw;
} gps_mavros__srv__GetGPSData_Response;

// Struct for a sequence of gps_mavros__srv__GetGPSData_Response.
typedef struct gps_mavros__srv__GetGPSData_Response__Sequence
{
  gps_mavros__srv__GetGPSData_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gps_mavros__srv__GetGPSData_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__STRUCT_H_
