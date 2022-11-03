// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/JoystickOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/joystick_order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_JoystickOrder_reverse
{
public:
  explicit Init_JoystickOrder_reverse(::interfaces::msg::JoystickOrder & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::JoystickOrder reverse(::interfaces::msg::JoystickOrder::_reverse_type arg)
  {
    msg_.reverse = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::JoystickOrder msg_;
};

class Init_JoystickOrder_steer
{
public:
  explicit Init_JoystickOrder_steer(::interfaces::msg::JoystickOrder & msg)
  : msg_(msg)
  {}
  Init_JoystickOrder_reverse steer(::interfaces::msg::JoystickOrder::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_JoystickOrder_reverse(msg_);
  }

private:
  ::interfaces::msg::JoystickOrder msg_;
};

class Init_JoystickOrder_throttle
{
public:
  explicit Init_JoystickOrder_throttle(::interfaces::msg::JoystickOrder & msg)
  : msg_(msg)
  {}
  Init_JoystickOrder_steer throttle(::interfaces::msg::JoystickOrder::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_JoystickOrder_steer(msg_);
  }

private:
  ::interfaces::msg::JoystickOrder msg_;
};

class Init_JoystickOrder_mode
{
public:
  explicit Init_JoystickOrder_mode(::interfaces::msg::JoystickOrder & msg)
  : msg_(msg)
  {}
  Init_JoystickOrder_throttle mode(::interfaces::msg::JoystickOrder::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_JoystickOrder_throttle(msg_);
  }

private:
  ::interfaces::msg::JoystickOrder msg_;
};

class Init_JoystickOrder_start
{
public:
  Init_JoystickOrder_start()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JoystickOrder_mode start(::interfaces::msg::JoystickOrder::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_JoystickOrder_mode(msg_);
  }

private:
  ::interfaces::msg::JoystickOrder msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::JoystickOrder>()
{
  return interfaces::msg::builder::Init_JoystickOrder_start();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__BUILDER_HPP_
