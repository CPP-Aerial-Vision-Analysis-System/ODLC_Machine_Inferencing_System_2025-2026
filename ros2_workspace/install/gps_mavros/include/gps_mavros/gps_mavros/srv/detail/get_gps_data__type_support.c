// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "gps_mavros/srv/detail/get_gps_data__rosidl_typesupport_introspection_c.h"
#include "gps_mavros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "gps_mavros/srv/detail/get_gps_data__functions.h"
#include "gps_mavros/srv/detail/get_gps_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gps_mavros__srv__GetGPSData_Request__init(message_memory);
}

void gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_fini_function(void * message_memory)
{
  gps_mavros__srv__GetGPSData_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gps_mavros__srv__GetGPSData_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_members = {
  "gps_mavros__srv",  // message namespace
  "GetGPSData_Request",  // message name
  1,  // number of fields
  sizeof(gps_mavros__srv__GetGPSData_Request),
  gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_member_array,  // message members
  gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_type_support_handle = {
  0,
  &gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gps_mavros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Request)() {
  if (!gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_type_support_handle.typesupport_identifier) {
    gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gps_mavros__srv__GetGPSData_Request__rosidl_typesupport_introspection_c__GetGPSData_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "gps_mavros/srv/detail/get_gps_data__rosidl_typesupport_introspection_c.h"
// already included above
// #include "gps_mavros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "gps_mavros/srv/detail/get_gps_data__functions.h"
// already included above
// #include "gps_mavros/srv/detail/get_gps_data__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  gps_mavros__srv__GetGPSData_Response__init(message_memory);
}

void gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_fini_function(void * message_memory)
{
  gps_mavros__srv__GetGPSData_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_member_array[4] = {
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gps_mavros__srv__GetGPSData_Response, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gps_mavros__srv__GetGPSData_Response, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gps_mavros__srv__GetGPSData_Response, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(gps_mavros__srv__GetGPSData_Response, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_members = {
  "gps_mavros__srv",  // message namespace
  "GetGPSData_Response",  // message name
  4,  // number of fields
  sizeof(gps_mavros__srv__GetGPSData_Response),
  gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_member_array,  // message members
  gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_type_support_handle = {
  0,
  &gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gps_mavros
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Response)() {
  if (!gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_type_support_handle.typesupport_identifier) {
    gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &gps_mavros__srv__GetGPSData_Response__rosidl_typesupport_introspection_c__GetGPSData_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "gps_mavros/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "gps_mavros/srv/detail/get_gps_data__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_members = {
  "gps_mavros__srv",  // service namespace
  "GetGPSData",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_Request_message_type_support_handle,
  NULL  // response message
  // gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_Response_message_type_support_handle
};

static rosidl_service_type_support_t gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_type_support_handle = {
  0,
  &gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_gps_mavros
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData)() {
  if (!gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_type_support_handle.typesupport_identifier) {
    gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, gps_mavros, srv, GetGPSData_Response)()->data;
  }

  return &gps_mavros__srv__detail__get_gps_data__rosidl_typesupport_introspection_c__GetGPSData_service_type_support_handle;
}
