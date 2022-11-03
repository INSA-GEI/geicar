// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/gnss__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Gnss_vacc
{
public:
  explicit Init_Gnss_vacc(::interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Gnss vacc(::interfaces::msg::Gnss::_vacc_type arg)
  {
    msg_.vacc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

class Init_Gnss_hacc
{
public:
  explicit Init_Gnss_hacc(::interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  Init_Gnss_vacc hacc(::interfaces::msg::Gnss::_hacc_type arg)
  {
    msg_.hacc = std::move(arg);
    return Init_Gnss_vacc(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

class Init_Gnss_quality
{
public:
  explicit Init_Gnss_quality(::interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  Init_Gnss_hacc quality(::interfaces::msg::Gnss::_quality_type arg)
  {
    msg_.quality = std::move(arg);
    return Init_Gnss_hacc(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

class Init_Gnss_altitude
{
public:
  explicit Init_Gnss_altitude(::interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  Init_Gnss_quality altitude(::interfaces::msg::Gnss::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_Gnss_quality(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

class Init_Gnss_longitude
{
public:
  explicit Init_Gnss_longitude(::interfaces::msg::Gnss & msg)
  : msg_(msg)
  {}
  Init_Gnss_altitude longitude(::interfaces::msg::Gnss::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_Gnss_altitude(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

class Init_Gnss_latitude
{
public:
  Init_Gnss_latitude()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gnss_longitude latitude(::interfaces::msg::Gnss::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_Gnss_longitude(msg_);
  }

private:
  ::interfaces::msg::Gnss msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Gnss>()
{
  return interfaces::msg::builder::Init_Gnss_latitude();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__GNSS__BUILDER_HPP_
