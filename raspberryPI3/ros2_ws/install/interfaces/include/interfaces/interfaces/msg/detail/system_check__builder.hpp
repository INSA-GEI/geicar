// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SYSTEM_CHECK__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__SYSTEM_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/system_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_SystemCheck_camera
{
public:
  explicit Init_SystemCheck_camera(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::SystemCheck camera(::interfaces::msg::SystemCheck::_camera_type arg)
  {
    msg_.camera = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_lidar
{
public:
  explicit Init_SystemCheck_lidar(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_camera lidar(::interfaces::msg::SystemCheck::_lidar_type arg)
  {
    msg_.lidar = std::move(arg);
    return Init_SystemCheck_camera(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_imu
{
public:
  explicit Init_SystemCheck_imu(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_lidar imu(::interfaces::msg::SystemCheck::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_SystemCheck_lidar(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_gps
{
public:
  explicit Init_SystemCheck_gps(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_imu gps(::interfaces::msg::SystemCheck::_gps_type arg)
  {
    msg_.gps = std::move(arg);
    return Init_SystemCheck_imu(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_ultrasonics
{
public:
  explicit Init_SystemCheck_ultrasonics(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_gps ultrasonics(::interfaces::msg::SystemCheck::_ultrasonics_type arg)
  {
    msg_.ultrasonics = std::move(arg);
    return Init_SystemCheck_gps(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_battery
{
public:
  explicit Init_SystemCheck_battery(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_ultrasonics battery(::interfaces::msg::SystemCheck::_battery_type arg)
  {
    msg_.battery = std::move(arg);
    return Init_SystemCheck_ultrasonics(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_comm_f103
{
public:
  explicit Init_SystemCheck_comm_f103(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_battery comm_f103(::interfaces::msg::SystemCheck::_comm_f103_type arg)
  {
    msg_.comm_f103 = std::move(arg);
    return Init_SystemCheck_battery(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_comm_l476
{
public:
  explicit Init_SystemCheck_comm_l476(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_comm_f103 comm_l476(::interfaces::msg::SystemCheck::_comm_l476_type arg)
  {
    msg_.comm_l476 = std::move(arg);
    return Init_SystemCheck_comm_f103(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_comm_jetson
{
public:
  explicit Init_SystemCheck_comm_jetson(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_comm_l476 comm_jetson(::interfaces::msg::SystemCheck::_comm_jetson_type arg)
  {
    msg_.comm_jetson = std::move(arg);
    return Init_SystemCheck_comm_l476(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_f103
{
public:
  explicit Init_SystemCheck_f103(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_comm_jetson f103(::interfaces::msg::SystemCheck::_f103_type arg)
  {
    msg_.f103 = std::move(arg);
    return Init_SystemCheck_comm_jetson(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_l476
{
public:
  explicit Init_SystemCheck_l476(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_f103 l476(::interfaces::msg::SystemCheck::_l476_type arg)
  {
    msg_.l476 = std::move(arg);
    return Init_SystemCheck_f103(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_jetson
{
public:
  explicit Init_SystemCheck_jetson(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_l476 jetson(::interfaces::msg::SystemCheck::_jetson_type arg)
  {
    msg_.jetson = std::move(arg);
    return Init_SystemCheck_l476(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_print
{
public:
  explicit Init_SystemCheck_print(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_jetson print(::interfaces::msg::SystemCheck::_print_type arg)
  {
    msg_.print = std::move(arg);
    return Init_SystemCheck_jetson(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_report
{
public:
  explicit Init_SystemCheck_report(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_print report(::interfaces::msg::SystemCheck::_report_type arg)
  {
    msg_.report = std::move(arg);
    return Init_SystemCheck_print(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_response
{
public:
  explicit Init_SystemCheck_response(::interfaces::msg::SystemCheck & msg)
  : msg_(msg)
  {}
  Init_SystemCheck_report response(::interfaces::msg::SystemCheck::_response_type arg)
  {
    msg_.response = std::move(arg);
    return Init_SystemCheck_report(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

class Init_SystemCheck_request
{
public:
  Init_SystemCheck_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemCheck_response request(::interfaces::msg::SystemCheck::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SystemCheck_response(msg_);
  }

private:
  ::interfaces::msg::SystemCheck msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::SystemCheck>()
{
  return interfaces::msg::builder::Init_SystemCheck_request();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__SYSTEM_CHECK__BUILDER_HPP_
