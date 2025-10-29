// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from temperature_interfaces:srv/ConvertTemperature.idl
// generated code does not contain a copyright notice
#include "temperature_interfaces/srv/detail/convert_temperature__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `conversion_type`
#include "rosidl_runtime_c/string_functions.h"

bool
temperature_interfaces__srv__ConvertTemperature_Request__init(temperature_interfaces__srv__ConvertTemperature_Request * msg)
{
  if (!msg) {
    return false;
  }
  // input_temp
  // conversion_type
  if (!rosidl_runtime_c__String__init(&msg->conversion_type)) {
    temperature_interfaces__srv__ConvertTemperature_Request__fini(msg);
    return false;
  }
  return true;
}

void
temperature_interfaces__srv__ConvertTemperature_Request__fini(temperature_interfaces__srv__ConvertTemperature_Request * msg)
{
  if (!msg) {
    return;
  }
  // input_temp
  // conversion_type
  rosidl_runtime_c__String__fini(&msg->conversion_type);
}

bool
temperature_interfaces__srv__ConvertTemperature_Request__are_equal(const temperature_interfaces__srv__ConvertTemperature_Request * lhs, const temperature_interfaces__srv__ConvertTemperature_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // input_temp
  if (lhs->input_temp != rhs->input_temp) {
    return false;
  }
  // conversion_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->conversion_type), &(rhs->conversion_type)))
  {
    return false;
  }
  return true;
}

bool
temperature_interfaces__srv__ConvertTemperature_Request__copy(
  const temperature_interfaces__srv__ConvertTemperature_Request * input,
  temperature_interfaces__srv__ConvertTemperature_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // input_temp
  output->input_temp = input->input_temp;
  // conversion_type
  if (!rosidl_runtime_c__String__copy(
      &(input->conversion_type), &(output->conversion_type)))
  {
    return false;
  }
  return true;
}

temperature_interfaces__srv__ConvertTemperature_Request *
temperature_interfaces__srv__ConvertTemperature_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Request * msg = (temperature_interfaces__srv__ConvertTemperature_Request *)allocator.allocate(sizeof(temperature_interfaces__srv__ConvertTemperature_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(temperature_interfaces__srv__ConvertTemperature_Request));
  bool success = temperature_interfaces__srv__ConvertTemperature_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
temperature_interfaces__srv__ConvertTemperature_Request__destroy(temperature_interfaces__srv__ConvertTemperature_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    temperature_interfaces__srv__ConvertTemperature_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__init(temperature_interfaces__srv__ConvertTemperature_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Request * data = NULL;

  if (size) {
    data = (temperature_interfaces__srv__ConvertTemperature_Request *)allocator.zero_allocate(size, sizeof(temperature_interfaces__srv__ConvertTemperature_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = temperature_interfaces__srv__ConvertTemperature_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        temperature_interfaces__srv__ConvertTemperature_Request__fini(&data[i - 1]);
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
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__fini(temperature_interfaces__srv__ConvertTemperature_Request__Sequence * array)
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
      temperature_interfaces__srv__ConvertTemperature_Request__fini(&array->data[i]);
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

temperature_interfaces__srv__ConvertTemperature_Request__Sequence *
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Request__Sequence * array = (temperature_interfaces__srv__ConvertTemperature_Request__Sequence *)allocator.allocate(sizeof(temperature_interfaces__srv__ConvertTemperature_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = temperature_interfaces__srv__ConvertTemperature_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__destroy(temperature_interfaces__srv__ConvertTemperature_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    temperature_interfaces__srv__ConvertTemperature_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__are_equal(const temperature_interfaces__srv__ConvertTemperature_Request__Sequence * lhs, const temperature_interfaces__srv__ConvertTemperature_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!temperature_interfaces__srv__ConvertTemperature_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
temperature_interfaces__srv__ConvertTemperature_Request__Sequence__copy(
  const temperature_interfaces__srv__ConvertTemperature_Request__Sequence * input,
  temperature_interfaces__srv__ConvertTemperature_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(temperature_interfaces__srv__ConvertTemperature_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    temperature_interfaces__srv__ConvertTemperature_Request * data =
      (temperature_interfaces__srv__ConvertTemperature_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!temperature_interfaces__srv__ConvertTemperature_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          temperature_interfaces__srv__ConvertTemperature_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!temperature_interfaces__srv__ConvertTemperature_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
temperature_interfaces__srv__ConvertTemperature_Response__init(temperature_interfaces__srv__ConvertTemperature_Response * msg)
{
  if (!msg) {
    return false;
  }
  // converted_temp
  return true;
}

void
temperature_interfaces__srv__ConvertTemperature_Response__fini(temperature_interfaces__srv__ConvertTemperature_Response * msg)
{
  if (!msg) {
    return;
  }
  // converted_temp
}

bool
temperature_interfaces__srv__ConvertTemperature_Response__are_equal(const temperature_interfaces__srv__ConvertTemperature_Response * lhs, const temperature_interfaces__srv__ConvertTemperature_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // converted_temp
  if (lhs->converted_temp != rhs->converted_temp) {
    return false;
  }
  return true;
}

bool
temperature_interfaces__srv__ConvertTemperature_Response__copy(
  const temperature_interfaces__srv__ConvertTemperature_Response * input,
  temperature_interfaces__srv__ConvertTemperature_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // converted_temp
  output->converted_temp = input->converted_temp;
  return true;
}

temperature_interfaces__srv__ConvertTemperature_Response *
temperature_interfaces__srv__ConvertTemperature_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Response * msg = (temperature_interfaces__srv__ConvertTemperature_Response *)allocator.allocate(sizeof(temperature_interfaces__srv__ConvertTemperature_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(temperature_interfaces__srv__ConvertTemperature_Response));
  bool success = temperature_interfaces__srv__ConvertTemperature_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
temperature_interfaces__srv__ConvertTemperature_Response__destroy(temperature_interfaces__srv__ConvertTemperature_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    temperature_interfaces__srv__ConvertTemperature_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__init(temperature_interfaces__srv__ConvertTemperature_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Response * data = NULL;

  if (size) {
    data = (temperature_interfaces__srv__ConvertTemperature_Response *)allocator.zero_allocate(size, sizeof(temperature_interfaces__srv__ConvertTemperature_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = temperature_interfaces__srv__ConvertTemperature_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        temperature_interfaces__srv__ConvertTemperature_Response__fini(&data[i - 1]);
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
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__fini(temperature_interfaces__srv__ConvertTemperature_Response__Sequence * array)
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
      temperature_interfaces__srv__ConvertTemperature_Response__fini(&array->data[i]);
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

temperature_interfaces__srv__ConvertTemperature_Response__Sequence *
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  temperature_interfaces__srv__ConvertTemperature_Response__Sequence * array = (temperature_interfaces__srv__ConvertTemperature_Response__Sequence *)allocator.allocate(sizeof(temperature_interfaces__srv__ConvertTemperature_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = temperature_interfaces__srv__ConvertTemperature_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__destroy(temperature_interfaces__srv__ConvertTemperature_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    temperature_interfaces__srv__ConvertTemperature_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__are_equal(const temperature_interfaces__srv__ConvertTemperature_Response__Sequence * lhs, const temperature_interfaces__srv__ConvertTemperature_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!temperature_interfaces__srv__ConvertTemperature_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
temperature_interfaces__srv__ConvertTemperature_Response__Sequence__copy(
  const temperature_interfaces__srv__ConvertTemperature_Response__Sequence * input,
  temperature_interfaces__srv__ConvertTemperature_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(temperature_interfaces__srv__ConvertTemperature_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    temperature_interfaces__srv__ConvertTemperature_Response * data =
      (temperature_interfaces__srv__ConvertTemperature_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!temperature_interfaces__srv__ConvertTemperature_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          temperature_interfaces__srv__ConvertTemperature_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!temperature_interfaces__srv__ConvertTemperature_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
