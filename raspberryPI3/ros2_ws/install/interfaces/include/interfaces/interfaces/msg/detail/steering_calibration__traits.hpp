// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/SteeringCalibration.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/steering_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SteeringCalibration & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << ", ";
  }

  // member: in_progress
  {
    out << "in_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.in_progress, out);
    out << ", ";
  }

  // member: user_need
  {
    out << "user_need: ";
    rosidl_generator_traits::value_to_yaml(msg.user_need, out);
    out << ", ";
  }

  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SteeringCalibration & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << "\n";
  }

  // member: in_progress
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_progress: ";
    rosidl_generator_traits::value_to_yaml(msg.in_progress, out);
    out << "\n";
  }

  // member: user_need
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "user_need: ";
    rosidl_generator_traits::value_to_yaml(msg.user_need, out);
    out << "\n";
  }

  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SteeringCalibration & msg, bool use_flow_style = false)
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
  const interfaces::msg::SteeringCalibration & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::SteeringCalibration & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::SteeringCalibration>()
{
  return "interfaces::msg::SteeringCalibration";
}

template<>
inline const char * name<interfaces::msg::SteeringCalibration>()
{
  return "interfaces/msg/SteeringCalibration";
}

template<>
struct has_fixed_size<interfaces::msg::SteeringCalibration>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::msg::SteeringCalibration>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::msg::SteeringCalibration>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__TRAITS_HPP_
