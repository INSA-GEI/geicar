// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice
#include "interfaces/msg/detail/system_check__rosidl_typesupport_fastrtps_cpp.hpp"
#include "interfaces/msg/detail/system_check__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interfaces
cdr_serialize(
  const interfaces::msg::SystemCheck & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: request
  cdr << (ros_message.request ? true : false);
  // Member: response
  cdr << (ros_message.response ? true : false);
  // Member: report
  cdr << (ros_message.report ? true : false);
  // Member: print
  cdr << (ros_message.print ? true : false);
  // Member: jetson
  cdr << (ros_message.jetson ? true : false);
  // Member: l476
  cdr << (ros_message.l476 ? true : false);
  // Member: f103
  cdr << (ros_message.f103 ? true : false);
  // Member: comm_jetson
  cdr << ros_message.comm_jetson;
  // Member: comm_l476
  cdr << ros_message.comm_l476;
  // Member: comm_f103
  cdr << ros_message.comm_f103;
  // Member: battery
  cdr << ros_message.battery;
  // Member: ultrasonics
  {
    cdr << ros_message.ultrasonics;
  }
  // Member: gps
  cdr << ros_message.gps;
  // Member: imu
  cdr << ros_message.imu;
  // Member: lidar
  cdr << ros_message.lidar;
  // Member: camera
  cdr << ros_message.camera;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  interfaces::msg::SystemCheck & ros_message)
{
  // Member: request
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.request = tmp ? true : false;
  }

  // Member: response
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.response = tmp ? true : false;
  }

  // Member: report
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.report = tmp ? true : false;
  }

  // Member: print
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.print = tmp ? true : false;
  }

  // Member: jetson
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.jetson = tmp ? true : false;
  }

  // Member: l476
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.l476 = tmp ? true : false;
  }

  // Member: f103
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.f103 = tmp ? true : false;
  }

  // Member: comm_jetson
  cdr >> ros_message.comm_jetson;

  // Member: comm_l476
  cdr >> ros_message.comm_l476;

  // Member: comm_f103
  cdr >> ros_message.comm_f103;

  // Member: battery
  cdr >> ros_message.battery;

  // Member: ultrasonics
  {
    cdr >> ros_message.ultrasonics;
  }

  // Member: gps
  cdr >> ros_message.gps;

  // Member: imu
  cdr >> ros_message.imu;

  // Member: lidar
  cdr >> ros_message.lidar;

  // Member: camera
  cdr >> ros_message.camera;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interfaces
get_serialized_size(
  const interfaces::msg::SystemCheck & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: request
  {
    size_t item_size = sizeof(ros_message.request);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: response
  {
    size_t item_size = sizeof(ros_message.response);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: report
  {
    size_t item_size = sizeof(ros_message.report);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: print
  {
    size_t item_size = sizeof(ros_message.print);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: jetson
  {
    size_t item_size = sizeof(ros_message.jetson);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l476
  {
    size_t item_size = sizeof(ros_message.l476);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: f103
  {
    size_t item_size = sizeof(ros_message.f103);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: comm_jetson
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.comm_jetson.size() + 1);
  // Member: comm_l476
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.comm_l476.size() + 1);
  // Member: comm_f103
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.comm_f103.size() + 1);
  // Member: battery
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.battery.size() + 1);
  // Member: ultrasonics
  {
    size_t array_size = 6;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.ultrasonics[index].size() + 1);
    }
  }
  // Member: gps
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.gps.size() + 1);
  // Member: imu
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.imu.size() + 1);
  // Member: lidar
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.lidar.size() + 1);
  // Member: camera
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.camera.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_interfaces
max_serialized_size_SystemCheck(
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


  // Member: request
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: response
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: report
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: print
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: jetson
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: l476
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: f103
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: comm_jetson
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

  // Member: comm_l476
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

  // Member: comm_f103
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

  // Member: battery
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

  // Member: ultrasonics
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

  // Member: gps
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

  // Member: imu
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

  // Member: lidar
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

  // Member: camera
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

static bool _SystemCheck__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const interfaces::msg::SystemCheck *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SystemCheck__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<interfaces::msg::SystemCheck *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SystemCheck__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const interfaces::msg::SystemCheck *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SystemCheck__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_SystemCheck(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _SystemCheck__callbacks = {
  "interfaces::msg",
  "SystemCheck",
  _SystemCheck__cdr_serialize,
  _SystemCheck__cdr_deserialize,
  _SystemCheck__get_serialized_size,
  _SystemCheck__max_serialized_size
};

static rosidl_message_type_support_t _SystemCheck__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SystemCheck__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<interfaces::msg::SystemCheck>()
{
  return &interfaces::msg::typesupport_fastrtps_cpp::_SystemCheck__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, interfaces, msg, SystemCheck)() {
  return &interfaces::msg::typesupport_fastrtps_cpp::_SystemCheck__handle;
}

#ifdef __cplusplus
}
#endif
