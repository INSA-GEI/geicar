// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/JoystickOrder.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/joystick_order__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces__msg__JoystickOrder__init(interfaces__msg__JoystickOrder * msg)
{
  if (!msg) {
    return false;
  }
  // start
  msg->start = false;
  // mode
  msg->mode = 0;
  // throttle
  msg->throttle = 0.0f;
  // steer
  msg->steer = 0.0f;
  // reverse
  msg->reverse = false;
  return true;
}

void
interfaces__msg__JoystickOrder__fini(interfaces__msg__JoystickOrder * msg)
{
  if (!msg) {
    return;
  }
  // start
  // mode
  // throttle
  // steer
  // reverse
}

bool
interfaces__msg__JoystickOrder__are_equal(const interfaces__msg__JoystickOrder * lhs, const interfaces__msg__JoystickOrder * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // start
  if (lhs->start != rhs->start) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // steer
  if (lhs->steer != rhs->steer) {
    return false;
  }
  // reverse
  if (lhs->reverse != rhs->reverse) {
    return false;
  }
  return true;
}

bool
interfaces__msg__JoystickOrder__copy(
  const interfaces__msg__JoystickOrder * input,
  interfaces__msg__JoystickOrder * output)
{
  if (!input || !output) {
    return false;
  }
  // start
  output->start = input->start;
  // mode
  output->mode = input->mode;
  // throttle
  output->throttle = input->throttle;
  // steer
  output->steer = input->steer;
  // reverse
  output->reverse = input->reverse;
  return true;
}

interfaces__msg__JoystickOrder *
interfaces__msg__JoystickOrder__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__JoystickOrder * msg = (interfaces__msg__JoystickOrder *)allocator.allocate(sizeof(interfaces__msg__JoystickOrder), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__JoystickOrder));
  bool success = interfaces__msg__JoystickOrder__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__JoystickOrder__destroy(interfaces__msg__JoystickOrder * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__JoystickOrder__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__JoystickOrder__Sequence__init(interfaces__msg__JoystickOrder__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__JoystickOrder * data = NULL;

  if (size) {
    data = (interfaces__msg__JoystickOrder *)allocator.zero_allocate(size, sizeof(interfaces__msg__JoystickOrder), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__JoystickOrder__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__JoystickOrder__fini(&data[i - 1]);
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
interfaces__msg__JoystickOrder__Sequence__fini(interfaces__msg__JoystickOrder__Sequence * array)
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
      interfaces__msg__JoystickOrder__fini(&array->data[i]);
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

interfaces__msg__JoystickOrder__Sequence *
interfaces__msg__JoystickOrder__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__JoystickOrder__Sequence * array = (interfaces__msg__JoystickOrder__Sequence *)allocator.allocate(sizeof(interfaces__msg__JoystickOrder__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__JoystickOrder__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__JoystickOrder__Sequence__destroy(interfaces__msg__JoystickOrder__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__JoystickOrder__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__JoystickOrder__Sequence__are_equal(const interfaces__msg__JoystickOrder__Sequence * lhs, const interfaces__msg__JoystickOrder__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__JoystickOrder__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__JoystickOrder__Sequence__copy(
  const interfaces__msg__JoystickOrder__Sequence * input,
  interfaces__msg__JoystickOrder__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__JoystickOrder);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__JoystickOrder * data =
      (interfaces__msg__JoystickOrder *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__JoystickOrder__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__JoystickOrder__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__JoystickOrder__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
