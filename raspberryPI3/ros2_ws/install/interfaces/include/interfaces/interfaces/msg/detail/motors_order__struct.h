// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/MotorsOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_H_
#define INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorsOrder in the package interfaces.
typedef struct interfaces__msg__MotorsOrder
{
  /// PWM
  int8_t right_rear_pwm;
  /// PWM
  int8_t left_rear_pwm;
  /// PWM
  int8_t steering_pwm;
} interfaces__msg__MotorsOrder;

// Struct for a sequence of interfaces__msg__MotorsOrder.
typedef struct interfaces__msg__MotorsOrder__Sequence
{
  interfaces__msg__MotorsOrder * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__MotorsOrder__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_H_
