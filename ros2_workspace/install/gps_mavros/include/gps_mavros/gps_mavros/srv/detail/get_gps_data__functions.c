// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from gps_mavros:srv/GetGPSData.idl
// generated code does not contain a copyright notice
#include "gps_mavros/srv/detail/get_gps_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
gps_mavros__srv__GetGPSData_Request__init(gps_mavros__srv__GetGPSData_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
gps_mavros__srv__GetGPSData_Request__fini(gps_mavros__srv__GetGPSData_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
gps_mavros__srv__GetGPSData_Request__are_equal(const gps_mavros__srv__GetGPSData_Request * lhs, const gps_mavros__srv__GetGPSData_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
gps_mavros__srv__GetGPSData_Request__copy(
  const gps_mavros__srv__GetGPSData_Request * input,
  gps_mavros__srv__GetGPSData_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

gps_mavros__srv__GetGPSData_Request *
gps_mavros__srv__GetGPSData_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Request * msg = (gps_mavros__srv__GetGPSData_Request *)allocator.allocate(sizeof(gps_mavros__srv__GetGPSData_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gps_mavros__srv__GetGPSData_Request));
  bool success = gps_mavros__srv__GetGPSData_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gps_mavros__srv__GetGPSData_Request__destroy(gps_mavros__srv__GetGPSData_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gps_mavros__srv__GetGPSData_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gps_mavros__srv__GetGPSData_Request__Sequence__init(gps_mavros__srv__GetGPSData_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Request * data = NULL;

  if (size) {
    data = (gps_mavros__srv__GetGPSData_Request *)allocator.zero_allocate(size, sizeof(gps_mavros__srv__GetGPSData_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gps_mavros__srv__GetGPSData_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gps_mavros__srv__GetGPSData_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gps_mavros__srv__GetGPSData_Request__Sequence__fini(gps_mavros__srv__GetGPSData_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gps_mavros__srv__GetGPSData_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gps_mavros__srv__GetGPSData_Request__Sequence *
gps_mavros__srv__GetGPSData_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Request__Sequence * array = (gps_mavros__srv__GetGPSData_Request__Sequence *)allocator.allocate(sizeof(gps_mavros__srv__GetGPSData_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gps_mavros__srv__GetGPSData_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gps_mavros__srv__GetGPSData_Request__Sequence__destroy(gps_mavros__srv__GetGPSData_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gps_mavros__srv__GetGPSData_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gps_mavros__srv__GetGPSData_Request__Sequence__are_equal(const gps_mavros__srv__GetGPSData_Request__Sequence * lhs, const gps_mavros__srv__GetGPSData_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gps_mavros__srv__GetGPSData_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gps_mavros__srv__GetGPSData_Request__Sequence__copy(
  const gps_mavros__srv__GetGPSData_Request__Sequence * input,
  gps_mavros__srv__GetGPSData_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gps_mavros__srv__GetGPSData_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gps_mavros__srv__GetGPSData_Request * data =
      (gps_mavros__srv__GetGPSData_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gps_mavros__srv__GetGPSData_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gps_mavros__srv__GetGPSData_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gps_mavros__srv__GetGPSData_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
gps_mavros__srv__GetGPSData_Response__init(gps_mavros__srv__GetGPSData_Response * msg)
{
  if (!msg) {
    return false;
  }
  // latitude
  // longitude
  // altitude
  // yaw
  return true;
}

void
gps_mavros__srv__GetGPSData_Response__fini(gps_mavros__srv__GetGPSData_Response * msg)
{
  if (!msg) {
    return;
  }
  // latitude
  // longitude
  // altitude
  // yaw
}

bool
gps_mavros__srv__GetGPSData_Response__are_equal(const gps_mavros__srv__GetGPSData_Response * lhs, const gps_mavros__srv__GetGPSData_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // latitude
  if (lhs->latitude != rhs->latitude) {
    return false;
  }
  // longitude
  if (lhs->longitude != rhs->longitude) {
    return false;
  }
  // altitude
  if (lhs->altitude != rhs->altitude) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
gps_mavros__srv__GetGPSData_Response__copy(
  const gps_mavros__srv__GetGPSData_Response * input,
  gps_mavros__srv__GetGPSData_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // latitude
  output->latitude = input->latitude;
  // longitude
  output->longitude = input->longitude;
  // altitude
  output->altitude = input->altitude;
  // yaw
  output->yaw = input->yaw;
  return true;
}

gps_mavros__srv__GetGPSData_Response *
gps_mavros__srv__GetGPSData_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Response * msg = (gps_mavros__srv__GetGPSData_Response *)allocator.allocate(sizeof(gps_mavros__srv__GetGPSData_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(gps_mavros__srv__GetGPSData_Response));
  bool success = gps_mavros__srv__GetGPSData_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
gps_mavros__srv__GetGPSData_Response__destroy(gps_mavros__srv__GetGPSData_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    gps_mavros__srv__GetGPSData_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
gps_mavros__srv__GetGPSData_Response__Sequence__init(gps_mavros__srv__GetGPSData_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Response * data = NULL;

  if (size) {
    data = (gps_mavros__srv__GetGPSData_Response *)allocator.zero_allocate(size, sizeof(gps_mavros__srv__GetGPSData_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = gps_mavros__srv__GetGPSData_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        gps_mavros__srv__GetGPSData_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
gps_mavros__srv__GetGPSData_Response__Sequence__fini(gps_mavros__srv__GetGPSData_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      gps_mavros__srv__GetGPSData_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

gps_mavros__srv__GetGPSData_Response__Sequence *
gps_mavros__srv__GetGPSData_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  gps_mavros__srv__GetGPSData_Response__Sequence * array = (gps_mavros__srv__GetGPSData_Response__Sequence *)allocator.allocate(sizeof(gps_mavros__srv__GetGPSData_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = gps_mavros__srv__GetGPSData_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
gps_mavros__srv__GetGPSData_Response__Sequence__destroy(gps_mavros__srv__GetGPSData_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    gps_mavros__srv__GetGPSData_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
gps_mavros__srv__GetGPSData_Response__Sequence__are_equal(const gps_mavros__srv__GetGPSData_Response__Sequence * lhs, const gps_mavros__srv__GetGPSData_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!gps_mavros__srv__GetGPSData_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
gps_mavros__srv__GetGPSData_Response__Sequence__copy(
  const gps_mavros__srv__GetGPSData_Response__Sequence * input,
  gps_mavros__srv__GetGPSData_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(gps_mavros__srv__GetGPSData_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    gps_mavros__srv__GetGPSData_Response * data =
      (gps_mavros__srv__GetGPSData_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!gps_mavros__srv__GetGPSData_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          gps_mavros__srv__GetGPSData_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!gps_mavros__srv__GetGPSData_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
