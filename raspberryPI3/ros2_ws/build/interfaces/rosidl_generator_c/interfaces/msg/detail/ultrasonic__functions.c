// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/Ultrasonic.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/ultrasonic__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces__msg__Ultrasonic__init(interfaces__msg__Ultrasonic * msg)
{
  if (!msg) {
    return false;
  }
  // front_left
  // front_center
  // front_right
  // rear_left
  // rear_center
  // rear_right
  return true;
}

void
interfaces__msg__Ultrasonic__fini(interfaces__msg__Ultrasonic * msg)
{
  if (!msg) {
    return;
  }
  // front_left
  // front_center
  // front_right
  // rear_left
  // rear_center
  // rear_right
}

bool
interfaces__msg__Ultrasonic__are_equal(const interfaces__msg__Ultrasonic * lhs, const interfaces__msg__Ultrasonic * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front_left
  if (lhs->front_left != rhs->front_left) {
    return false;
  }
  // front_center
  if (lhs->front_center != rhs->front_center) {
    return false;
  }
  // front_right
  if (lhs->front_right != rhs->front_right) {
    return false;
  }
  // rear_left
  if (lhs->rear_left != rhs->rear_left) {
    return false;
  }
  // rear_center
  if (lhs->rear_center != rhs->rear_center) {
    return false;
  }
  // rear_right
  if (lhs->rear_right != rhs->rear_right) {
    return false;
  }
  return true;
}

bool
interfaces__msg__Ultrasonic__copy(
  const interfaces__msg__Ultrasonic * input,
  interfaces__msg__Ultrasonic * output)
{
  if (!input || !output) {
    return false;
  }
  // front_left
  output->front_left = input->front_left;
  // front_center
  output->front_center = input->front_center;
  // front_right
  output->front_right = input->front_right;
  // rear_left
  output->rear_left = input->rear_left;
  // rear_center
  output->rear_center = input->rear_center;
  // rear_right
  output->rear_right = input->rear_right;
  return true;
}

interfaces__msg__Ultrasonic *
interfaces__msg__Ultrasonic__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Ultrasonic * msg = (interfaces__msg__Ultrasonic *)allocator.allocate(sizeof(interfaces__msg__Ultrasonic), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__Ultrasonic));
  bool success = interfaces__msg__Ultrasonic__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__Ultrasonic__destroy(interfaces__msg__Ultrasonic * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__Ultrasonic__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__Ultrasonic__Sequence__init(interfaces__msg__Ultrasonic__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Ultrasonic * data = NULL;

  if (size) {
    data = (interfaces__msg__Ultrasonic *)allocator.zero_allocate(size, sizeof(interfaces__msg__Ultrasonic), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__Ultrasonic__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__Ultrasonic__fini(&data[i - 1]);
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
interfaces__msg__Ultrasonic__Sequence__fini(interfaces__msg__Ultrasonic__Sequence * array)
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
      interfaces__msg__Ultrasonic__fini(&array->data[i]);
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

interfaces__msg__Ultrasonic__Sequence *
interfaces__msg__Ultrasonic__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__Ultrasonic__Sequence * array = (interfaces__msg__Ultrasonic__Sequence *)allocator.allocate(sizeof(interfaces__msg__Ultrasonic__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__Ultrasonic__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__Ultrasonic__Sequence__destroy(interfaces__msg__Ultrasonic__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__Ultrasonic__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__Ultrasonic__Sequence__are_equal(const interfaces__msg__Ultrasonic__Sequence * lhs, const interfaces__msg__Ultrasonic__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__Ultrasonic__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__Ultrasonic__Sequence__copy(
  const interfaces__msg__Ultrasonic__Sequence * input,
  interfaces__msg__Ultrasonic__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__Ultrasonic);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__Ultrasonic * data =
      (interfaces__msg__Ultrasonic *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__Ultrasonic__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__Ultrasonic__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__Ultrasonic__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
