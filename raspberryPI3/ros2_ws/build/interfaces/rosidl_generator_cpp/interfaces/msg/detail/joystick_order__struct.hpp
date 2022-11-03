// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/JoystickOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__JoystickOrder __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__JoystickOrder __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JoystickOrder_
{
  using Type = JoystickOrder_<ContainerAllocator>;

  explicit JoystickOrder_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->start = false;
      this->mode = 0;
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->reverse = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->start = false;
      this->mode = 0;
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->reverse = false;
    }
  }

  explicit JoystickOrder_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->start = false;
      this->mode = 0;
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->reverse = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->start = false;
      this->mode = 0;
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->reverse = false;
    }
  }

  // field types and members
  using _start_type =
    bool;
  _start_type start;
  using _mode_type =
    int8_t;
  _mode_type mode;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _steer_type =
    float;
  _steer_type steer;
  using _reverse_type =
    bool;
  _reverse_type reverse;

  // setters for named parameter idiom
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__mode(
    const int8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__steer(
    const float & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__reverse(
    const bool & _arg)
  {
    this->reverse = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::JoystickOrder_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::JoystickOrder_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::JoystickOrder_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::JoystickOrder_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__JoystickOrder
    std::shared_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__JoystickOrder
    std::shared_ptr<interfaces::msg::JoystickOrder_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JoystickOrder_ & other) const
  {
    if (this->start != other.start) {
      return false;
    }
    if (this->mode != other.mode) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    if (this->reverse != other.reverse) {
      return false;
    }
    return true;
  }
  bool operator!=(const JoystickOrder_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JoystickOrder_

// alias to use template instance with default allocator
using JoystickOrder =
  interfaces::msg::JoystickOrder_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__JOYSTICK_ORDER__STRUCT_HPP_
