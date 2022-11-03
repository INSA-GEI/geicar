// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/system_check__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `comm_jetson`
// Member `comm_l476`
// Member `comm_f103`
// Member `battery`
// Member `ultrasonics`
// Member `gps`
// Member `imu`
// Member `lidar`
// Member `camera`
#include "rosidl_runtime_c/string_functions.h"

bool
interfaces__msg__SystemCheck__init(interfaces__msg__SystemCheck * msg)
{
  if (!msg) {
    return false;
  }
  // request
  msg->request = false;
  // response
  msg->response = false;
  // report
  msg->report = false;
  // print
  msg->print = false;
  // jetson
  msg->jetson = false;
  // l476
  msg->l476 = false;
  // f103
  msg->f103 = false;
  // comm_jetson
  if (!rosidl_runtime_c__String__init(&msg->comm_jetson)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // comm_l476
  if (!rosidl_runtime_c__String__init(&msg->comm_l476)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // comm_f103
  if (!rosidl_runtime_c__String__init(&msg->comm_f103)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // battery
  if (!rosidl_runtime_c__String__init(&msg->battery)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // ultrasonics
  for (size_t i = 0; i < 6; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->ultrasonics[i])) {
      interfaces__msg__SystemCheck__fini(msg);
      return false;
    }
  }
  // gps
  if (!rosidl_runtime_c__String__init(&msg->gps)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // imu
  if (!rosidl_runtime_c__String__init(&msg->imu)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // lidar
  if (!rosidl_runtime_c__String__init(&msg->lidar)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  // camera
  if (!rosidl_runtime_c__String__init(&msg->camera)) {
    interfaces__msg__SystemCheck__fini(msg);
    return false;
  }
  return true;
}

void
interfaces__msg__SystemCheck__fini(interfaces__msg__SystemCheck * msg)
{
  if (!msg) {
    return;
  }
  // request
  // response
  // report
  // print
  // jetson
  // l476
  // f103
  // comm_jetson
  rosidl_runtime_c__String__fini(&msg->comm_jetson);
  // comm_l476
  rosidl_runtime_c__String__fini(&msg->comm_l476);
  // comm_f103
  rosidl_runtime_c__String__fini(&msg->comm_f103);
  // battery
  rosidl_runtime_c__String__fini(&msg->battery);
  // ultrasonics
  for (size_t i = 0; i < 6; ++i) {
    rosidl_runtime_c__String__fini(&msg->ultrasonics[i]);
  }
  // gps
  rosidl_runtime_c__String__fini(&msg->gps);
  // imu
  rosidl_runtime_c__String__fini(&msg->imu);
  // lidar
  rosidl_runtime_c__String__fini(&msg->lidar);
  // camera
  rosidl_runtime_c__String__fini(&msg->camera);
}

bool
interfaces__msg__SystemCheck__are_equal(const interfaces__msg__SystemCheck * lhs, const interfaces__msg__SystemCheck * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // request
  if (lhs->request != rhs->request) {
    return false;
  }
  // response
  if (lhs->response != rhs->response) {
    return false;
  }
  // report
  if (lhs->report != rhs->report) {
    return false;
  }
  // print
  if (lhs->print != rhs->print) {
    return false;
  }
  // jetson
  if (lhs->jetson != rhs->jetson) {
    return false;
  }
  // l476
  if (lhs->l476 != rhs->l476) {
    return false;
  }
  // f103
  if (lhs->f103 != rhs->f103) {
    return false;
  }
  // comm_jetson
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->comm_jetson), &(rhs->comm_jetson)))
  {
    return false;
  }
  // comm_l476
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->comm_l476), &(rhs->comm_l476)))
  {
    return false;
  }
  // comm_f103
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->comm_f103), &(rhs->comm_f103)))
  {
    return false;
  }
  // battery
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->battery), &(rhs->battery)))
  {
    return false;
  }
  // ultrasonics
  for (size_t i = 0; i < 6; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->ultrasonics[i]), &(rhs->ultrasonics[i])))
    {
      return false;
    }
  }
  // gps
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->gps), &(rhs->gps)))
  {
    return false;
  }
  // imu
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->imu), &(rhs->imu)))
  {
    return false;
  }
  // lidar
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->lidar), &(rhs->lidar)))
  {
    return false;
  }
  // camera
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->camera), &(rhs->camera)))
  {
    return false;
  }
  return true;
}

bool
interfaces__msg__SystemCheck__copy(
  const interfaces__msg__SystemCheck * input,
  interfaces__msg__SystemCheck * output)
{
  if (!input || !output) {
    return false;
  }
  // request
  output->request = input->request;
  // response
  output->response = input->response;
  // report
  output->report = input->report;
  // print
  output->print = input->print;
  // jetson
  output->jetson = input->jetson;
  // l476
  output->l476 = input->l476;
  // f103
  output->f103 = input->f103;
  // comm_jetson
  if (!rosidl_runtime_c__String__copy(
      &(input->comm_jetson), &(output->comm_jetson)))
  {
    return false;
  }
  // comm_l476
  if (!rosidl_runtime_c__String__copy(
      &(input->comm_l476), &(output->comm_l476)))
  {
    return false;
  }
  // comm_f103
  if (!rosidl_runtime_c__String__copy(
      &(input->comm_f103), &(output->comm_f103)))
  {
    return false;
  }
  // battery
  if (!rosidl_runtime_c__String__copy(
      &(input->battery), &(output->battery)))
  {
    return false;
  }
  // ultrasonics
  for (size_t i = 0; i < 6; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->ultrasonics[i]), &(output->ultrasonics[i])))
    {
      return false;
    }
  }
  // gps
  if (!rosidl_runtime_c__String__copy(
      &(input->gps), &(output->gps)))
  {
    return false;
  }
  // imu
  if (!rosidl_runtime_c__String__copy(
      &(input->imu), &(output->imu)))
  {
    return false;
  }
  // lidar
  if (!rosidl_runtime_c__String__copy(
      &(input->lidar), &(output->lidar)))
  {
    return false;
  }
  // camera
  if (!rosidl_runtime_c__String__copy(
      &(input->camera), &(output->camera)))
  {
    return false;
  }
  return true;
}

interfaces__msg__SystemCheck *
interfaces__msg__SystemCheck__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SystemCheck * msg = (interfaces__msg__SystemCheck *)allocator.allocate(sizeof(interfaces__msg__SystemCheck), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(interfaces__msg__SystemCheck));
  bool success = interfaces__msg__SystemCheck__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
interfaces__msg__SystemCheck__destroy(interfaces__msg__SystemCheck * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    interfaces__msg__SystemCheck__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
interfaces__msg__SystemCheck__Sequence__init(interfaces__msg__SystemCheck__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SystemCheck * data = NULL;

  if (size) {
    data = (interfaces__msg__SystemCheck *)allocator.zero_allocate(size, sizeof(interfaces__msg__SystemCheck), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = interfaces__msg__SystemCheck__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        interfaces__msg__SystemCheck__fini(&data[i - 1]);
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
interfaces__msg__SystemCheck__Sequence__fini(interfaces__msg__SystemCheck__Sequence * array)
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
      interfaces__msg__SystemCheck__fini(&array->data[i]);
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

interfaces__msg__SystemCheck__Sequence *
interfaces__msg__SystemCheck__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  interfaces__msg__SystemCheck__Sequence * array = (interfaces__msg__SystemCheck__Sequence *)allocator.allocate(sizeof(interfaces__msg__SystemCheck__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = interfaces__msg__SystemCheck__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
interfaces__msg__SystemCheck__Sequence__destroy(interfaces__msg__SystemCheck__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    interfaces__msg__SystemCheck__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
interfaces__msg__SystemCheck__Sequence__are_equal(const interfaces__msg__SystemCheck__Sequence * lhs, const interfaces__msg__SystemCheck__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!interfaces__msg__SystemCheck__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
interfaces__msg__SystemCheck__Sequence__copy(
  const interfaces__msg__SystemCheck__Sequence * input,
  interfaces__msg__SystemCheck__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(interfaces__msg__SystemCheck);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    interfaces__msg__SystemCheck * data =
      (interfaces__msg__SystemCheck *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!interfaces__msg__SystemCheck__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          interfaces__msg__SystemCheck__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!interfaces__msg__SystemCheck__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
