// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/GeneralData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GENERAL_DATA__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__GENERAL_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/general_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_GeneralData_humidity
{
public:
  explicit Init_GeneralData_humidity(::interfaces::msg::GeneralData & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::GeneralData humidity(::interfaces::msg::GeneralData::_humidity_type arg)
  {
    msg_.humidity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::GeneralData msg_;
};

class Init_GeneralData_pressure
{
public:
  explicit Init_GeneralData_pressure(::interfaces::msg::GeneralData & msg)
  : msg_(msg)
  {}
  Init_GeneralData_humidity pressure(::interfaces::msg::GeneralData::_pressure_type arg)
  {
    msg_.pressure = std::move(arg);
    return Init_GeneralData_humidity(msg_);
  }

private:
  ::interfaces::msg::GeneralData msg_;
};

class Init_GeneralData_temperature
{
public:
  explicit Init_GeneralData_temperature(::interfaces::msg::GeneralData & msg)
  : msg_(msg)
  {}
  Init_GeneralData_pressure temperature(::interfaces::msg::GeneralData::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_GeneralData_pressure(msg_);
  }

private:
  ::interfaces::msg::GeneralData msg_;
};

class Init_GeneralData_battery_level
{
public:
  Init_GeneralData_battery_level()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GeneralData_temperature battery_level(::interfaces::msg::GeneralData::_battery_level_type arg)
  {
    msg_.battery_level = std::move(arg);
    return Init_GeneralData_temperature(msg_);
  }

private:
  ::interfaces::msg::GeneralData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::GeneralData>()
{
  return interfaces::msg::builder::Init_GeneralData_battery_level();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__GENERAL_DATA__BUILDER_HPP_
