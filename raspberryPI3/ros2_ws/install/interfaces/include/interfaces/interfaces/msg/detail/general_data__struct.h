// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/GeneralData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_H_
#define INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/GeneralData in the package interfaces.
typedef struct interfaces__msg__GeneralData
{
  float battery_level;
  float temperature;
  int16_t pressure;
  int8_t humidity;
} interfaces__msg__GeneralData;

// Struct for a sequence of interfaces__msg__GeneralData.
typedef struct interfaces__msg__GeneralData__Sequence
{
  interfaces__msg__GeneralData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__GeneralData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_H_
