// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/MotorsFeedback.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/motors_feedback__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_MotorsFeedback_steering_angle
{
public:
  explicit Init_MotorsFeedback_steering_angle(::interfaces::msg::MotorsFeedback & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::MotorsFeedback steering_angle(::interfaces::msg::MotorsFeedback::_steering_angle_type arg)
  {
    msg_.steering_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::MotorsFeedback msg_;
};

class Init_MotorsFeedback_right_rear_speed
{
public:
  explicit Init_MotorsFeedback_right_rear_speed(::interfaces::msg::MotorsFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorsFeedback_steering_angle right_rear_speed(::interfaces::msg::MotorsFeedback::_right_rear_speed_type arg)
  {
    msg_.right_rear_speed = std::move(arg);
    return Init_MotorsFeedback_steering_angle(msg_);
  }

private:
  ::interfaces::msg::MotorsFeedback msg_;
};

class Init_MotorsFeedback_left_rear_speed
{
public:
  explicit Init_MotorsFeedback_left_rear_speed(::interfaces::msg::MotorsFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorsFeedback_right_rear_speed left_rear_speed(::interfaces::msg::MotorsFeedback::_left_rear_speed_type arg)
  {
    msg_.left_rear_speed = std::move(arg);
    return Init_MotorsFeedback_right_rear_speed(msg_);
  }

private:
  ::interfaces::msg::MotorsFeedback msg_;
};

class Init_MotorsFeedback_right_rear_odometry
{
public:
  explicit Init_MotorsFeedback_right_rear_odometry(::interfaces::msg::MotorsFeedback & msg)
  : msg_(msg)
  {}
  Init_MotorsFeedback_left_rear_speed right_rear_odometry(::interfaces::msg::MotorsFeedback::_right_rear_odometry_type arg)
  {
    msg_.right_rear_odometry = std::move(arg);
    return Init_MotorsFeedback_left_rear_speed(msg_);
  }

private:
  ::interfaces::msg::MotorsFeedback msg_;
};

class Init_MotorsFeedback_left_rear_odometry
{
public:
  Init_MotorsFeedback_left_rear_odometry()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorsFeedback_right_rear_odometry left_rear_odometry(::interfaces::msg::MotorsFeedback::_left_rear_odometry_type arg)
  {
    msg_.left_rear_odometry = std::move(arg);
    return Init_MotorsFeedback_right_rear_odometry(msg_);
  }

private:
  ::interfaces::msg::MotorsFeedback msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::MotorsFeedback>()
{
  return interfaces::msg::builder::Init_MotorsFeedback_left_rear_odometry();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__BUILDER_HPP_
