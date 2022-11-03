// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/GeneralData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__GeneralData __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__GeneralData __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GeneralData_
{
  using Type = GeneralData_<ContainerAllocator>;

  explicit GeneralData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->temperature = 0.0f;
      this->pressure = 0;
      this->humidity = 0;
    }
  }

  explicit GeneralData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->battery_level = 0.0f;
      this->temperature = 0.0f;
      this->pressure = 0;
      this->humidity = 0;
    }
  }

  // field types and members
  using _battery_level_type =
    float;
  _battery_level_type battery_level;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _pressure_type =
    int16_t;
  _pressure_type pressure;
  using _humidity_type =
    int8_t;
  _humidity_type humidity;

  // setters for named parameter idiom
  Type & set__battery_level(
    const float & _arg)
  {
    this->battery_level = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__pressure(
    const int16_t & _arg)
  {
    this->pressure = _arg;
    return *this;
  }
  Type & set__humidity(
    const int8_t & _arg)
  {
    this->humidity = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::GeneralData_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::GeneralData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::GeneralData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::GeneralData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::GeneralData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::GeneralData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::GeneralData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::GeneralData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::GeneralData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::GeneralData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__GeneralData
    std::shared_ptr<interfaces::msg::GeneralData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__GeneralData
    std::shared_ptr<interfaces::msg::GeneralData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GeneralData_ & other) const
  {
    if (this->battery_level != other.battery_level) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->pressure != other.pressure) {
      return false;
    }
    if (this->humidity != other.humidity) {
      return false;
    }
    return true;
  }
  bool operator!=(const GeneralData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GeneralData_

// alias to use template instance with default allocator
using GeneralData =
  interfaces::msg::GeneralData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__GENERAL_DATA__STRUCT_HPP_
