// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/MotorsOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_ORDER__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_ORDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/motors_order__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorsOrder & msg,
  std::ostream & out)
{
  out << "{";
  // member: right_rear_pwm
  {
    out << "right_rear_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_pwm, out);
    out << ", ";
  }

  // member: left_rear_pwm
  {
    out << "left_rear_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_pwm, out);
    out << ", ";
  }

  // member: steering_pwm
  {
    out << "steering_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_pwm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorsOrder & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: right_rear_pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_rear_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.right_rear_pwm, out);
    out << "\n";
  }

  // member: left_rear_pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_rear_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.left_rear_pwm, out);
    out << "\n";
  }

  // member: steering_pwm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering_pwm: ";
    rosidl_generator_traits::value_to_yaml(msg.steering_pwm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorsOrder & msg, bool use_flow_style = false)
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
  const interfaces::msg::MotorsOrder & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::MotorsOrder & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::MotorsOrder>()
{
  return "interfaces::msg::MotorsOrder";
}

template<>
inline const char * name<interfaces::msg::MotorsOrder>()
{
  return "interfaces/msg/MotorsOrder";
}

template<>
struct has_fixed_size<interfaces::msg::MotorsOrder>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::MotorsOrder>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::MotorsOrder>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__MOTORS_ORDER__TRAITS_HPP_
