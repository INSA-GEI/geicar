// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/SteeringCalibration.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/steering_calibration__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces__msg__SteeringCalibration__init(interfaces__msg__SteeringCalibration * msg)
{
  if (!msg) {
    return false;
  }
  // request
  msg->request = false;
  // in_progress
  msg->in_progress = false;
  // user_need
  msg->user_need = false;
  // status
  msg->status = 0;
  return true;
}

void
interfaces__msg__SteeringCalibration__fini(interfaces__msg__SteeringCalibration * msg)
{
  if (!msg) {
    return;
  }
  // request
  // in_progress
  // user_need
  // status
}

bool
interfaces__msg__SteeringCalibration__are_equal(const interfaces__msg__SteeringCalibration * lhs, const interfaces__msg__SteeringCalibration * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request
  if (lhs->request != rhs->request) {
    return false;
  }
  // in_progress
  if (lhs->in_progress != rhs->in_progress) {
    return false;
  }
  // user_need
  if (lhs->user_need != rhs->user_need) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
interfaces__msg__SteeringCalibration__copy(
  const interfaces__msg__SteeringCalibration * input,
  interfaces__msg__SteeringCalibration * output)
{
  if (!input || !output) {
    return false;
  }
  // request
  output->request = input->request;
  // in_progress
  output->in_progress = input->in_progress;
  // user_need
  output->user_need = input->user_need;
  // status
  output->status = input->status;
  return true;
}

interfaces__msg__SteeringCalibration *
interfaces__msg__SteeringCalibration__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SteeringCalibration * msg = (interfaces__msg__SteeringCalibration *)allocator.allocate(sizeof(interfaces__msg__SteeringCalibration), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__SteeringCalibration));
  bool success = interfaces__msg__SteeringCalibration__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__SteeringCalibration__destroy(interfaces__msg__SteeringCalibration * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__SteeringCalibration__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__SteeringCalibration__Sequence__init(interfaces__msg__SteeringCalibration__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SteeringCalibration * data = NULL;

  if (size) {
    data = (interfaces__msg__SteeringCalibration *)allocator.zero_allocate(size, sizeof(interfaces__msg__SteeringCalibration), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__SteeringCalibration__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__SteeringCalibration__fini(&data[i - 1]);
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
interfaces__msg__SteeringCalibration__Sequence__fini(interfaces__msg__SteeringCalibration__Sequence * array)
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
      interfaces__msg__SteeringCalibration__fini(&array->data[i]);
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

interfaces__msg__SteeringCalibration__Sequence *
interfaces__msg__SteeringCalibration__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SteeringCalibration__Sequence * array = (interfaces__msg__SteeringCalibration__Sequence *)allocator.allocate(sizeof(interfaces__msg__SteeringCalibration__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__SteeringCalibration__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__SteeringCalibration__Sequence__destroy(interfaces__msg__SteeringCalibration__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__SteeringCalibration__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__SteeringCalibration__Sequence__are_equal(const interfaces__msg__SteeringCalibration__Sequence * lhs, const interfaces__msg__SteeringCalibration__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__SteeringCalibration__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__SteeringCalibration__Sequence__copy(
  const interfaces__msg__SteeringCalibration__Sequence * input,
  interfaces__msg__SteeringCalibration__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__SteeringCalibration);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__SteeringCalibration * data =
      (interfaces__msg__SteeringCalibration *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__SteeringCalibration__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__SteeringCalibration__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__SteeringCalibration__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
