// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SYSTEM_CHECK__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__SYSTEM_CHECK__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/system_check__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SystemCheck & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    out << "request: ";
    rosidl_generator_traits::value_to_yaml(msg.request, out);
    out << ", ";
  }

  // member: response
  {
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << ", ";
  }

  // member: report
  {
    out << "report: ";
    rosidl_generator_traits::value_to_yaml(msg.report, out);
    out << ", ";
  }

  // member: print
  {
    out << "print: ";
    rosidl_generator_traits::value_to_yaml(msg.print, out);
    out << ", ";
  }

  // member: jetson
  {
    out << "jetson: ";
    rosidl_generator_traits::value_to_yaml(msg.jetson, out);
    out << ", ";
  }

  // member: l476
  {
    out << "l476: ";
    rosidl_generator_traits::value_to_yaml(msg.l476, out);
    out << ", ";
  }

  // member: f103
  {
    out << "f103: ";
    rosidl_generator_traits::value_to_yaml(msg.f103, out);
    out << ", ";
  }

  // member: comm_jetson
  {
    out << "comm_jetson: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_jetson, out);
    out << ", ";
  }

  // member: comm_l476
  {
    out << "comm_l476: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_l476, out);
    out << ", ";
  }

  // member: comm_f103
  {
    out << "comm_f103: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_f103, out);
    out << ", ";
  }

  // member: battery
  {
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << ", ";
  }

  // member: ultrasonics
  {
    if (msg.ultrasonics.size() == 0) {
      out << "ultrasonics: []";
    } else {
      out << "ultrasonics: [";
      size_t pending_items = msg.ultrasonics.size();
      for (auto item : msg.ultrasonics) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps
  {
    out << "gps: ";
    rosidl_generator_traits::value_to_yaml(msg.gps, out);
    out << ", ";
  }

  // member: imu
  {
    out << "imu: ";
    rosidl_generator_traits::value_to_yaml(msg.imu, out);
    out << ", ";
  }

  // member: lidar
  {
    out << "lidar: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar, out);
    out << ", ";
  }

  // member: camera
  {
    out << "camera: ";
    rosidl_generator_traits::value_to_yaml(msg.camera, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SystemCheck & msg,
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

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "response: ";
    rosidl_generator_traits::value_to_yaml(msg.response, out);
    out << "\n";
  }

  // member: report
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "report: ";
    rosidl_generator_traits::value_to_yaml(msg.report, out);
    out << "\n";
  }

  // member: print
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "print: ";
    rosidl_generator_traits::value_to_yaml(msg.print, out);
    out << "\n";
  }

  // member: jetson
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jetson: ";
    rosidl_generator_traits::value_to_yaml(msg.jetson, out);
    out << "\n";
  }

  // member: l476
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "l476: ";
    rosidl_generator_traits::value_to_yaml(msg.l476, out);
    out << "\n";
  }

  // member: f103
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f103: ";
    rosidl_generator_traits::value_to_yaml(msg.f103, out);
    out << "\n";
  }

  // member: comm_jetson
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "comm_jetson: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_jetson, out);
    out << "\n";
  }

  // member: comm_l476
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "comm_l476: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_l476, out);
    out << "\n";
  }

  // member: comm_f103
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "comm_f103: ";
    rosidl_generator_traits::value_to_yaml(msg.comm_f103, out);
    out << "\n";
  }

  // member: battery
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery: ";
    rosidl_generator_traits::value_to_yaml(msg.battery, out);
    out << "\n";
  }

  // member: ultrasonics
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ultrasonics.size() == 0) {
      out << "ultrasonics: []\n";
    } else {
      out << "ultrasonics:\n";
      for (auto item : msg.ultrasonics) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps: ";
    rosidl_generator_traits::value_to_yaml(msg.gps, out);
    out << "\n";
  }

  // member: imu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "imu: ";
    rosidl_generator_traits::value_to_yaml(msg.imu, out);
    out << "\n";
  }

  // member: lidar
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lidar: ";
    rosidl_generator_traits::value_to_yaml(msg.lidar, out);
    out << "\n";
  }

  // member: camera
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "camera: ";
    rosidl_generator_traits::value_to_yaml(msg.camera, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SystemCheck & msg, bool use_flow_style = false)
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
  const interfaces::msg::SystemCheck & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::SystemCheck & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::SystemCheck>()
{
  return "interfaces::msg::SystemCheck";
}

template<>
inline const char * name<interfaces::msg::SystemCheck>()
{
  return "interfaces/msg/SystemCheck";
}

template<>
struct has_fixed_size<interfaces::msg::SystemCheck>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::SystemCheck>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::SystemCheck>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__SYSTEM_CHECK__TRAITS_HPP_
