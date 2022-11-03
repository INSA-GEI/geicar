// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Ultrasonic.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ULTRASONIC__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ULTRASONIC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Ultrasonic in the package interfaces.
typedef struct interfaces__msg__Ultrasonic
{
  int16_t front_left;
  int16_t front_center;
  int16_t front_right;
  int16_t rear_left;
  int16_t rear_center;
  int16_t rear_right;
} interfaces__msg__Ultrasonic;

// Struct for a sequence of interfaces__msg__Ultrasonic.
typedef struct interfaces__msg__Ultrasonic__Sequence
{
  interfaces__msg__Ultrasonic * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Ultrasonic__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ULTRASONIC__STRUCT_H_
