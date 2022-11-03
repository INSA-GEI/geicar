// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/SteeringCalibration.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/steering_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_SteeringCalibration_status
{
public:
  explicit Init_SteeringCalibration_status(::interfaces::msg::SteeringCalibration & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::SteeringCalibration status(::interfaces::msg::SteeringCalibration::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::SteeringCalibration msg_;
};

class Init_SteeringCalibration_user_need
{
public:
  explicit Init_SteeringCalibration_user_need(::interfaces::msg::SteeringCalibration & msg)
  : msg_(msg)
  {}
  Init_SteeringCalibration_status user_need(::interfaces::msg::SteeringCalibration::_user_need_type arg)
  {
    msg_.user_need = std::move(arg);
    return Init_SteeringCalibration_status(msg_);
  }

private:
  ::interfaces::msg::SteeringCalibration msg_;
};

class Init_SteeringCalibration_in_progress
{
public:
  explicit Init_SteeringCalibration_in_progress(::interfaces::msg::SteeringCalibration & msg)
  : msg_(msg)
  {}
  Init_SteeringCalibration_user_need in_progress(::interfaces::msg::SteeringCalibration::_in_progress_type arg)
  {
    msg_.in_progress = std::move(arg);
    return Init_SteeringCalibration_user_need(msg_);
  }

private:
  ::interfaces::msg::SteeringCalibration msg_;
};

class Init_SteeringCalibration_request
{
public:
  Init_SteeringCalibration_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SteeringCalibration_in_progress request(::interfaces::msg::SteeringCalibration::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SteeringCalibration_in_progress(msg_);
  }

private:
  ::interfaces::msg::SteeringCalibration msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::SteeringCalibration>()
{
  return interfaces::msg::builder::Init_SteeringCalibration_request();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__BUILDER_HPP_
