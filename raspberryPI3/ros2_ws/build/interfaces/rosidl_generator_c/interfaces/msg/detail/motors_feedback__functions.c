// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/MotorsFeedback.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/motors_feedback__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
interfaces__msg__MotorsFeedback__init(interfaces__msg__MotorsFeedback * msg)
{
  if (!msg) {
    return false;
  }
  // left_rear_odometry
  msg->left_rear_odometry = 0;
  // right_rear_odometry
  msg->right_rear_odometry = 0;
  // left_rear_speed
  msg->left_rear_speed = 0.0f;
  // right_rear_speed
  msg->right_rear_speed = 0.0f;
  // steering_angle
  msg->steering_angle = 0.0f;
  return true;
}

void
interfaces__msg__MotorsFeedback__fini(interfaces__msg__MotorsFeedback * msg)
{
  if (!msg) {
    return;
  }
  // left_rear_odometry
  // right_rear_odometry
  // left_rear_speed
  // right_rear_speed
  // steering_angle
}

bool
interfaces__msg__MotorsFeedback__are_equal(const interfaces__msg__MotorsFeedback * lhs, const interfaces__msg__MotorsFeedback * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // left_rear_odometry
  if (lhs->left_rear_odometry != rhs->left_rear_odometry) {
    return false;
  }
  // right_rear_odometry
  if (lhs->right_rear_odometry != rhs->right_rear_odometry) {
    return false;
  }
  // left_rear_speed
  if (lhs->left_rear_speed != rhs->left_rear_speed) {
    return false;
  }
  // right_rear_speed
  if (lhs->right_rear_speed != rhs->right_rear_speed) {
    return false;
  }
  // steering_angle
  if (lhs->steering_angle != rhs->steering_angle) {
    return false;
  }
  return true;
}

bool
interfaces__msg__MotorsFeedback__copy(
  const interfaces__msg__MotorsFeedback * input,
  interfaces__msg__MotorsFeedback * output)
{
  if (!input || !output) {
    return false;
  }
  // left_rear_odometry
  output->left_rear_odometry = input->left_rear_odometry;
  // right_rear_odometry
  output->right_rear_odometry = input->right_rear_odometry;
  // left_rear_speed
  output->left_rear_speed = input->left_rear_speed;
  // right_rear_speed
  output->right_rear_speed = input->right_rear_speed;
  // steering_angle
  output->steering_angle = input->steering_angle;
  return true;
}

interfaces__msg__MotorsFeedback *
interfaces__msg__MotorsFeedback__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__MotorsFeedback * msg = (interfaces__msg__MotorsFeedback *)allocator.allocate(sizeof(interfaces__msg__MotorsFeedback), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__MotorsFeedback));
  bool success = interfaces__msg__MotorsFeedback__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__MotorsFeedback__destroy(interfaces__msg__MotorsFeedback * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__MotorsFeedback__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__MotorsFeedback__Sequence__init(interfaces__msg__MotorsFeedback__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__MotorsFeedback * data = NULL;

  if (size) {
    data = (interfaces__msg__MotorsFeedback *)allocator.zero_allocate(size, sizeof(interfaces__msg__MotorsFeedback), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__MotorsFeedback__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__MotorsFeedback__fini(&data[i - 1]);
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
interfaces__msg__MotorsFeedback__Sequence__fini(interfaces__msg__MotorsFeedback__Sequence * array)
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
      interfaces__msg__MotorsFeedback__fini(&array->data[i]);
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

interfaces__msg__MotorsFeedback__Sequence *
interfaces__msg__MotorsFeedback__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__MotorsFeedback__Sequence * array = (interfaces__msg__MotorsFeedback__Sequence *)allocator.allocate(sizeof(interfaces__msg__MotorsFeedback__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__MotorsFeedback__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__MotorsFeedback__Sequence__destroy(interfaces__msg__MotorsFeedback__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__MotorsFeedback__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__MotorsFeedback__Sequence__are_equal(const interfaces__msg__MotorsFeedback__Sequence * lhs, const interfaces__msg__MotorsFeedback__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__MotorsFeedback__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__MotorsFeedback__Sequence__copy(
  const interfaces__msg__MotorsFeedback__Sequence * input,
  interfaces__msg__MotorsFeedback__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__MotorsFeedback);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__MotorsFeedback * data =
      (interfaces__msg__MotorsFeedback *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__MotorsFeedback__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__MotorsFeedback__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__MotorsFeedback__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
