// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/MotorsFeedback.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_H_
#define INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorsFeedback in the package interfaces.
typedef struct interfaces__msg__MotorsFeedback
{
  /// Number of pulses since last message (36 pulses <-> 1 revolution)
  int8_t left_rear_odometry;
  /// Number of pulses since last message (36 pulses <-> 1 revolution)
  int8_t right_rear_odometry;
  /// RPM
  float left_rear_speed;
  /// RPM
  float right_rear_speed;
  float steering_angle;
} interfaces__msg__MotorsFeedback;

// Struct for a sequence of interfaces__msg__MotorsFeedback.
typedef struct interfaces__msg__MotorsFeedback__Sequence
{
  interfaces__msg__MotorsFeedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__MotorsFeedback__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_H_
