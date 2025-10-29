// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "temperature_interfaces/srv/detail/convert_temperature__rosidl_typesupport_introspection_c.h"
#include "temperature_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "temperature_interfaces/srv/detail/convert_temperature__functions.h"
#include "temperature_interfaces/srv/detail/convert_temperature__struct.h"


// Include directives for member types
// Member `conversion_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  temperature_interfaces__srv__ConvertTemperature_Request__init(message_memory);
}

void temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_fini_function(void * message_memory)
{
  temperature_interfaces__srv__ConvertTemperature_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_member_array[2] = {
  {
    "input_temp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(temperature_interfaces__srv__ConvertTemperature_Request, input_temp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "conversion_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(temperature_interfaces__srv__ConvertTemperature_Request, conversion_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_members = {
  "temperature_interfaces__srv",  // message namespace
  "ConvertTemperature_Request",  // message name
  2,  // number of fields
  sizeof(temperature_interfaces__srv__ConvertTemperature_Request),
  temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_member_array,  // message members
  temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_type_support_handle = {
  0,
  &temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_temperature_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Request)() {
  if (!temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_type_support_handle.typesupport_identifier) {
    temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &temperature_interfaces__srv__ConvertTemperature_Request__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__rosidl_typesupport_introspection_c.h"
// already included above
// #include "temperature_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__functions.h"
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  temperature_interfaces__srv__ConvertTemperature_Response__init(message_memory);
}

void temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_fini_function(void * message_memory)
{
  temperature_interfaces__srv__ConvertTemperature_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_member_array[1] = {
  {
    "converted_temp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(temperature_interfaces__srv__ConvertTemperature_Response, converted_temp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_members = {
  "temperature_interfaces__srv",  // message namespace
  "ConvertTemperature_Response",  // message name
  1,  // number of fields
  sizeof(temperature_interfaces__srv__ConvertTemperature_Response),
  temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_member_array,  // message members
  temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_type_support_handle = {
  0,
  &temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_temperature_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Response)() {
  if (!temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_type_support_handle.typesupport_identifier) {
    temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &temperature_interfaces__srv__ConvertTemperature_Response__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "temperature_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "temperature_interfaces/srv/detail/convert_temperature__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_members = {
  "temperature_interfaces__srv",  // service namespace
  "ConvertTemperature",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_Request_message_type_support_handle,
  NULL  // response message
  // temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_Response_message_type_support_handle
};

static rosidl_service_type_support_t temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_type_support_handle = {
  0,
  &temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_temperature_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature)() {
  if (!temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_type_support_handle.typesupport_identifier) {
    temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, temperature_interfaces, srv, ConvertTemperature_Response)()->data;
  }

  return &temperature_interfaces__srv__detail__convert_temperature__rosidl_typesupport_introspection_c__ConvertTemperature_service_type_support_handle;
}
