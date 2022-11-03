// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/MotorsOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_ORDER__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/motors_order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorsOrder_steering_pwm
{
public:
  explicit Init_MotorsOrder_steering_pwm(::interfaces::msg::MotorsOrder & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::MotorsOrder steering_pwm(::interfaces::msg::MotorsOrder::_steering_pwm_type arg)
  {
    msg_.steering_pwm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::MotorsOrder msg_;
};

class Init_MotorsOrder_left_rear_pwm
{
public:
  explicit Init_MotorsOrder_left_rear_pwm(::interfaces::msg::MotorsOrder & msg)
  : msg_(msg)
  {}
  Init_MotorsOrder_steering_pwm left_rear_pwm(::interfaces::msg::MotorsOrder::_left_rear_pwm_type arg)
  {
    msg_.left_rear_pwm = std::move(arg);
    return Init_MotorsOrder_steering_pwm(msg_);
  }

private:
  ::interfaces::msg::MotorsOrder msg_;
};

class Init_MotorsOrder_right_rear_pwm
{
public:
  Init_MotorsOrder_right_rear_pwm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorsOrder_left_rear_pwm right_rear_pwm(::interfaces::msg::MotorsOrder::_right_rear_pwm_type arg)
  {
    msg_.right_rear_pwm = std::move(arg);
    return Init_MotorsOrder_left_rear_pwm(msg_);
  }

private:
  ::interfaces::msg::MotorsOrder msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::MotorsOrder>()
{
  return interfaces::msg::builder::Init_MotorsOrder_right_rear_pwm();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__MOTORS_ORDER__BUILDER_HPP_
