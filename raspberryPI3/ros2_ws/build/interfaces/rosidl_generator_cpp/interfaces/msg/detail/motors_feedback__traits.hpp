// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/MotorsFeedback.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/motors_feedback__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorsFeedback & msg,
  std::ostream & out)
{
  out << "{";
  // member: left_rear_odometry
  {
    out << "left_rear_odometry: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_odometry, out);
    out << ", ";
  }

  // member: right_rear_odometry
  {
    out << "right_rear_odometry: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_odometry, out);
    out << ", ";
  }

  // member: left_rear_speed
  {
    out << "left_rear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_speed, out);
    out << ", ";
  }

  // member: right_rear_speed
  {
    out << "right_rear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_speed, out);
    out << ", ";
  }

  // member: steering_angle
  {
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorsFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: left_rear_odometry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_odometry: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_odometry, out);
    out << "\n";
  }

  // member: right_rear_odometry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_odometry: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_odometry, out);
    out << "\n";
  }

  // member: left_rear_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_speed, out);
    out << "\n";
  }

  // member: right_rear_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_speed, out);
    out << "\n";
  }

  // member: steering_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorsFeedback & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::MotorsFeedback & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::MotorsFeedback & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::MotorsFeedback>()
{
  return "interfaces::msg::MotorsFeedback";
}

template<>
inline const char * name<interfaces::msg::MotorsFeedback>()
{
  return "interfaces/msg/MotorsFeedback";
}

template<>
struct has_fixed_size<interfaces::msg::MotorsFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::MotorsFeedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::MotorsFeedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__TRAITS_HPP_
