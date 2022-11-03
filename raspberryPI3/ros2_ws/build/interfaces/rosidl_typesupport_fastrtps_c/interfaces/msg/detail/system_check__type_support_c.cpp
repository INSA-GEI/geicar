// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/system_check__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "interfaces/msg/detail/system_check__struct.h"
#include "interfaces/msg/detail/system_check__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // battery, camera, comm_f103, comm_jetson, comm_l476, gps, imu, lidar, ultrasonics
#include "rosidl_runtime_c/string_functions.h"  // battery, camera, comm_f103, comm_jetson, comm_l476, gps, imu, lidar, ultrasonics

// forward declare type support functions


using _SystemCheck__ros_msg_type = interfaces__msg__SystemCheck;

static bool _SystemCheck__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SystemCheck__ros_msg_type * ros_message = static_cast<const _SystemCheck__ros_msg_type *>(untyped_ros_message);
  // Field name: request
  {
    cdr << (ros_message->request ? true : false);
  }

  // Field name: response
  {
    cdr << (ros_message->response ? true : false);
  }

  // Field name: report
  {
    cdr << (ros_message->report ? true : false);
  }

  // Field name: print
  {
    cdr << (ros_message->print ? true : false);
  }

  // Field name: jetson
  {
    cdr << (ros_message->jetson ? true : false);
  }

  // Field name: l476
  {
    cdr << (ros_message->l476 ? true : false);
  }

  // Field name: f103
  {
    cdr << (ros_message->f103 ? true : false);
  }

  // Field name: comm_jetson
  {
    const rosidl_runtime_c__String * str = &ros_message->comm_jetson;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: comm_l476
  {
    const rosidl_runtime_c__String * str = &ros_message->comm_l476;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: comm_f103
  {
    const rosidl_runtime_c__String * str = &ros_message->comm_f103;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: battery
  {
    const rosidl_runtime_c__String * str = &ros_message->battery;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: ultrasonics
  {
    size_t size = 6;
    auto array_ptr = ros_message->ultrasonics;
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: gps
  {
    const rosidl_runtime_c__String * str = &ros_message->gps;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: imu
  {
    const rosidl_runtime_c__String * str = &ros_message->imu;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: lidar
  {
    const rosidl_runtime_c__String * str = &ros_message->lidar;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: camera
  {
    const rosidl_runtime_c__String * str = &ros_message->camera;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SystemCheck__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SystemCheck__ros_msg_type * ros_message = static_cast<_SystemCheck__ros_msg_type *>(untyped_ros_message);
  // Field name: request
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->request = tmp ? true : false;
  }

  // Field name: response
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->response = tmp ? true : false;
  }

  // Field name: report
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->report = tmp ? true : false;
  }

  // Field name: print
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->print = tmp ? true : false;
  }

  // Field name: jetson
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->jetson = tmp ? true : false;
  }

  // Field name: l476
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->l476 = tmp ? true : false;
  }

  // Field name: f103
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->f103 = tmp ? true : false;
  }

  // Field name: comm_jetson
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->comm_jetson.data) {
      rosidl_runtime_c__String__init(&ros_message->comm_jetson);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->comm_jetson,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'comm_jetson'\n");
      return false;
    }
  }

  // Field name: comm_l476
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->comm_l476.data) {
      rosidl_runtime_c__String__init(&ros_message->comm_l476);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->comm_l476,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'comm_l476'\n");
      return false;
    }
  }

  // Field name: comm_f103
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->comm_f103.data) {
      rosidl_runtime_c__String__init(&ros_message->comm_f103);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->comm_f103,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'comm_f103'\n");
      return false;
    }
  }

  // Field name: battery
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->battery.data) {
      rosidl_runtime_c__String__init(&ros_message->battery);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->battery,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'battery'\n");
      return false;
    }
  }

  // Field name: ultrasonics
  {
    size_t size = 6;
    auto array_ptr = ros_message->ultrasonics;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'ultrasonics'\n");
        return false;
      }
    }
  }

  // Field name: gps
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->gps.data) {
      rosidl_runtime_c__String__init(&ros_message->gps);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->gps,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'gps'\n");
      return false;
    }
  }

  // Field name: imu
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->imu.data) {
      rosidl_runtime_c__String__init(&ros_message->imu);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->imu,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'imu'\n");
      return false;
    }
  }

  // Field name: lidar
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->lidar.data) {
      rosidl_runtime_c__String__init(&ros_message->lidar);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->lidar,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'lidar'\n");
      return false;
    }
  }

  // Field name: camera
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->camera.data) {
      rosidl_runtime_c__String__init(&ros_message->camera);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->camera,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'camera'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t get_serialized_size_interfaces__msg__SystemCheck(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SystemCheck__ros_msg_type * ros_message = static_cast<const _SystemCheck__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name request
  {
    size_t item_size = sizeof(ros_message->request);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name response
  {
    size_t item_size = sizeof(ros_message->response);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name report
  {
    size_t item_size = sizeof(ros_message->report);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name print
  {
    size_t item_size = sizeof(ros_message->print);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name jetson
  {
    size_t item_size = sizeof(ros_message->jetson);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l476
  {
    size_t item_size = sizeof(ros_message->l476);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name f103
  {
    size_t item_size = sizeof(ros_message->f103);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name comm_jetson
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->comm_jetson.size + 1);
  // field.name comm_l476
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->comm_l476.size + 1);
  // field.name comm_f103
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->comm_f103.size + 1);
  // field.name battery
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->battery.size + 1);
  // field.name ultrasonics
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->ultrasonics;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name gps
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->gps.size + 1);
  // field.name imu
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->imu.size + 1);
  // field.name lidar
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->lidar.size + 1);
  // field.name camera
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->camera.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SystemCheck__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_interfaces__msg__SystemCheck(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_interfaces
size_t max_serialized_size_interfaces__msg__SystemCheck(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: request
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: response
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: report
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: print
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: jetson
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: l476
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: f103
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: comm_jetson
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: comm_l476
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: comm_f103
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: battery
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: ultrasonics
  {
    size_t array_size = 6;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: gps
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: imu
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: lidar
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: camera
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SystemCheck__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_interfaces__msg__SystemCheck(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SystemCheck = {
  "interfaces::msg",
  "SystemCheck",
  _SystemCheck__cdr_serialize,
  _SystemCheck__cdr_deserialize,
  _SystemCheck__get_serialized_size,
  _SystemCheck__max_serialized_size
};

static rosidl_message_type_support_t _SystemCheck__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SystemCheck,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, interfaces, msg, SystemCheck)() {
  return &_SystemCheck__type_support;
}

#if defined(__cplusplus)
}
#endif
