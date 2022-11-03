// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_H_
#define INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'comm_jetson'
// Member 'comm_l476'
// Member 'comm_f103'
// Member 'battery'
// Member 'ultrasonics'
// Member 'gps'
// Member 'imu'
// Member 'lidar'
// Member 'camera'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SystemCheck in the package interfaces.
/**
  * Message status
 */
typedef struct interfaces__msg__SystemCheck
{
  /// If true, the message is a request to check communications
  bool request;
  /// If true, the message is a response to the previous request
  bool response;
  /// If true, the message is a report published by the system_check_node
  bool report;
  /// If true, the message is a request to print the last report in the terminal
  bool print;
  /// responses
  bool jetson;
  bool l476;
  bool f103;
  /// report
  /// "OK" or "Failed"
  rosidl_runtime_c__String comm_jetson;
  /// "OK" or "Failed"
  rosidl_runtime_c__String comm_l476;
  /// "OK" or "Failed"
  rosidl_runtime_c__String comm_f103;
  /// "OK", "Low Voltage", "High Voltage", "No data"
  rosidl_runtime_c__String battery;
  /// [Front left, Front center, Front right, Rear left, Rear center, Rear right], possible values : "OK", "Out of range", "No data"
  rosidl_runtime_c__String ultrasonics[6];
  /// "No Fix", "Autonomous GNSS fix", "Differential GNSS fix", "RTK fixed", "RTK float", "Estimated/dead reckoning fix", "No data"
  rosidl_runtime_c__String gps;
  /// "OK", "No data"
  rosidl_runtime_c__String imu;
  /// "OK", "No data"
  rosidl_runtime_c__String lidar;
  /// "OK", "No data"
  rosidl_runtime_c__String camera;
} interfaces__msg__SystemCheck;

// Struct for a sequence of interfaces__msg__SystemCheck.
typedef struct interfaces__msg__SystemCheck__Sequence
{
  interfaces__msg__SystemCheck * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__SystemCheck__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_H_
