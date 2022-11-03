// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Ultrasonic.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ULTRASONIC__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ULTRASONIC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/ultrasonic__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Ultrasonic_rear_right
{
public:
  explicit Init_Ultrasonic_rear_right(::interfaces::msg::Ultrasonic & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Ultrasonic rear_right(::interfaces::msg::Ultrasonic::_rear_right_type arg)
  {
    msg_.rear_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

class Init_Ultrasonic_rear_center
{
public:
  explicit Init_Ultrasonic_rear_center(::interfaces::msg::Ultrasonic & msg)
  : msg_(msg)
  {}
  Init_Ultrasonic_rear_right rear_center(::interfaces::msg::Ultrasonic::_rear_center_type arg)
  {
    msg_.rear_center = std::move(arg);
    return Init_Ultrasonic_rear_right(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

class Init_Ultrasonic_rear_left
{
public:
  explicit Init_Ultrasonic_rear_left(::interfaces::msg::Ultrasonic & msg)
  : msg_(msg)
  {}
  Init_Ultrasonic_rear_center rear_left(::interfaces::msg::Ultrasonic::_rear_left_type arg)
  {
    msg_.rear_left = std::move(arg);
    return Init_Ultrasonic_rear_center(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

class Init_Ultrasonic_front_right
{
public:
  explicit Init_Ultrasonic_front_right(::interfaces::msg::Ultrasonic & msg)
  : msg_(msg)
  {}
  Init_Ultrasonic_rear_left front_right(::interfaces::msg::Ultrasonic::_front_right_type arg)
  {
    msg_.front_right = std::move(arg);
    return Init_Ultrasonic_rear_left(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

class Init_Ultrasonic_front_center
{
public:
  explicit Init_Ultrasonic_front_center(::interfaces::msg::Ultrasonic & msg)
  : msg_(msg)
  {}
  Init_Ultrasonic_front_right front_center(::interfaces::msg::Ultrasonic::_front_center_type arg)
  {
    msg_.front_center = std::move(arg);
    return Init_Ultrasonic_front_right(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

class Init_Ultrasonic_front_left
{
public:
  Init_Ultrasonic_front_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Ultrasonic_front_center front_left(::interfaces::msg::Ultrasonic::_front_left_type arg)
  {
    msg_.front_left = std::move(arg);
    return Init_Ultrasonic_front_center(msg_);
  }

private:
  ::interfaces::msg::Ultrasonic msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Ultrasonic>()
{
  return interfaces::msg::builder::Init_Ultrasonic_front_left();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ULTRASONIC__BUILDER_HPP_
