// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/SteeringCalibration.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_H_
#define INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/SteeringCalibration in the package interfaces.
typedef struct interfaces__msg__SteeringCalibration
{
  bool request;
  bool in_progress;
  bool user_need;
  /// 1 = SUCCESS / -1 = FAILED
  int8_t status;
} interfaces__msg__SteeringCalibration;

// Struct for a sequence of interfaces__msg__SteeringCalibration.
typedef struct interfaces__msg__SteeringCalibration__Sequence
{
  interfaces__msg__SteeringCalibration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__SteeringCalibration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_H_
