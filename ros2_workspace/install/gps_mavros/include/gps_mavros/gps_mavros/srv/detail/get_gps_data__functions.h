// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice

#ifndef GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__FUNCTIONS_H_
#define GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "gps_mavros/msg/rosidl_generator_c__visibility_control.h"

#include "gps_mavros/srv/detail/get_gps_data__struct.h"

/// Initialize srv/GetGPSData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gps_mavros__srv__GetGPSData_Request
 * )) before or use
 * gps_mavros__srv__GetGPSData_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__init(gps_mavros__srv__GetGPSData_Request * msg);

/// Finalize srv/GetGPSData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Request__fini(gps_mavros__srv__GetGPSData_Request * msg);

/// Create srv/GetGPSData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gps_mavros__srv__GetGPSData_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
gps_mavros__srv__GetGPSData_Request *
gps_mavros__srv__GetGPSData_Request__create();

/// Destroy srv/GetGPSData message.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Request__destroy(gps_mavros__srv__GetGPSData_Request * msg);

/// Check for srv/GetGPSData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__are_equal(const gps_mavros__srv__GetGPSData_Request * lhs, const gps_mavros__srv__GetGPSData_Request * rhs);

/// Copy a srv/GetGPSData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__copy(
  const gps_mavros__srv__GetGPSData_Request * input,
  gps_mavros__srv__GetGPSData_Request * output);

/// Initialize array of srv/GetGPSData messages.
/**
 * It allocates the memory for the number of elements and calls
 * gps_mavros__srv__GetGPSData_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__Sequence__init(gps_mavros__srv__GetGPSData_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetGPSData messages.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Request__Sequence__fini(gps_mavros__srv__GetGPSData_Request__Sequence * array);

/// Create array of srv/GetGPSData messages.
/**
 * It allocates the memory for the array and calls
 * gps_mavros__srv__GetGPSData_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
gps_mavros__srv__GetGPSData_Request__Sequence *
gps_mavros__srv__GetGPSData_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetGPSData messages.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Request__Sequence__destroy(gps_mavros__srv__GetGPSData_Request__Sequence * array);

/// Check for srv/GetGPSData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__Sequence__are_equal(const gps_mavros__srv__GetGPSData_Request__Sequence * lhs, const gps_mavros__srv__GetGPSData_Request__Sequence * rhs);

/// Copy an array of srv/GetGPSData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Request__Sequence__copy(
  const gps_mavros__srv__GetGPSData_Request__Sequence * input,
  gps_mavros__srv__GetGPSData_Request__Sequence * output);

/// Initialize srv/GetGPSData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * gps_mavros__srv__GetGPSData_Response
 * )) before or use
 * gps_mavros__srv__GetGPSData_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__init(gps_mavros__srv__GetGPSData_Response * msg);

/// Finalize srv/GetGPSData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Response__fini(gps_mavros__srv__GetGPSData_Response * msg);

/// Create srv/GetGPSData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * gps_mavros__srv__GetGPSData_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
gps_mavros__srv__GetGPSData_Response *
gps_mavros__srv__GetGPSData_Response__create();

/// Destroy srv/GetGPSData message.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Response__destroy(gps_mavros__srv__GetGPSData_Response * msg);

/// Check for srv/GetGPSData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__are_equal(const gps_mavros__srv__GetGPSData_Response * lhs, const gps_mavros__srv__GetGPSData_Response * rhs);

/// Copy a srv/GetGPSData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__copy(
  const gps_mavros__srv__GetGPSData_Response * input,
  gps_mavros__srv__GetGPSData_Response * output);

/// Initialize array of srv/GetGPSData messages.
/**
 * It allocates the memory for the number of elements and calls
 * gps_mavros__srv__GetGPSData_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__Sequence__init(gps_mavros__srv__GetGPSData_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetGPSData messages.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Response__Sequence__fini(gps_mavros__srv__GetGPSData_Response__Sequence * array);

/// Create array of srv/GetGPSData messages.
/**
 * It allocates the memory for the array and calls
 * gps_mavros__srv__GetGPSData_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
gps_mavros__srv__GetGPSData_Response__Sequence *
gps_mavros__srv__GetGPSData_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetGPSData messages.
/**
 * It calls
 * gps_mavros__srv__GetGPSData_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
void
gps_mavros__srv__GetGPSData_Response__Sequence__destroy(gps_mavros__srv__GetGPSData_Response__Sequence * array);

/// Check for srv/GetGPSData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__Sequence__are_equal(const gps_mavros__srv__GetGPSData_Response__Sequence * lhs, const gps_mavros__srv__GetGPSData_Response__Sequence * rhs);

/// Copy an array of srv/GetGPSData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_gps_mavros
bool
gps_mavros__srv__GetGPSData_Response__Sequence__copy(
  const gps_mavros__srv__GetGPSData_Response__Sequence * input,
  gps_mavros__srv__GetGPSData_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // GPS_MAVROS__SRV__DETAIL__GET_GPS_DATA__FUNCTIONS_H_
