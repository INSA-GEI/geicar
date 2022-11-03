// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/JoystickOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_H_
#define INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/JoystickOrder in the package interfaces.
/**
  * General orders
 */
typedef struct interfaces__msg__JoystickOrder
{
  bool start;
  /// 0 -> Manual ; 1 -> Autonomous ; 2 -> Steering Calibration
  int8_t mode;
  /// Manual orders
  /// [0 ; 1]
  float throttle;
  float steer;
  bool reverse;
} interfaces__msg__JoystickOrder;

// Struct for a sequence of interfaces__msg__JoystickOrder.
typedef struct interfaces__msg__JoystickOrder__Sequence
{
  interfaces__msg__JoystickOrder * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__JoystickOrder__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_H_
