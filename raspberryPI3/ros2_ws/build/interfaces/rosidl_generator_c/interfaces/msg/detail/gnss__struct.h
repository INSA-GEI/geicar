// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
#define INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Gnss in the package interfaces.
typedef struct interfaces__msg__Gnss
{
  double latitude;
  double longitude;
  double altitude;
  /// 0 = no fix, 1 = Autonomous GNSS fix, 2 = Differential GNSS fix, 4 = RTK Fixed, 5 = RTK Float, 6 = estimated/dead reckoning fix
  int8_t quality;
  /// Horizontal Accuracy
  double hacc;
  /// Vertical Accuracy
  double vacc;
} interfaces__msg__Gnss;

// Struct for a sequence of interfaces__msg__Gnss.
typedef struct interfaces__msg__Gnss__Sequence
{
  interfaces__msg__Gnss * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Gnss__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__GNSS__STRUCT_H_
