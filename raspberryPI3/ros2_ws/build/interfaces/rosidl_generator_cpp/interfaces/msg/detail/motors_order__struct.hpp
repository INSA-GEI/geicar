// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/MotorsOrder.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__MotorsOrder __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__MotorsOrder __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorsOrder_
{
  using Type = MotorsOrder_<ContainerAllocator>;

  explicit MotorsOrder_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->right_rear_pwm = 50;
      this->left_rear_pwm = 50;
      this->steering_pwm = 50;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->right_rear_pwm = 0;
      this->left_rear_pwm = 0;
      this->steering_pwm = 0;
    }
  }

  explicit MotorsOrder_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->right_rear_pwm = 50;
      this->left_rear_pwm = 50;
      this->steering_pwm = 50;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->right_rear_pwm = 0;
      this->left_rear_pwm = 0;
      this->steering_pwm = 0;
    }
  }

  // field types and members
  using _right_rear_pwm_type =
    int8_t;
  _right_rear_pwm_type right_rear_pwm;
  using _left_rear_pwm_type =
    int8_t;
  _left_rear_pwm_type left_rear_pwm;
  using _steering_pwm_type =
    int8_t;
  _steering_pwm_type steering_pwm;

  // setters for named parameter idiom
  Type & set__right_rear_pwm(
    const int8_t & _arg)
  {
    this->right_rear_pwm = _arg;
    return *this;
  }
  Type & set__left_rear_pwm(
    const int8_t & _arg)
  {
    this->left_rear_pwm = _arg;
    return *this;
  }
  Type & set__steering_pwm(
    const int8_t & _arg)
  {
    this->steering_pwm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::MotorsOrder_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::MotorsOrder_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::MotorsOrder_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::MotorsOrder_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__MotorsOrder
    std::shared_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__MotorsOrder
    std::shared_ptr<interfaces::msg::MotorsOrder_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorsOrder_ & other) const
  {
    if (this->right_rear_pwm != other.right_rear_pwm) {
      return false;
    }
    if (this->left_rear_pwm != other.left_rear_pwm) {
      return false;
    }
    if (this->steering_pwm != other.steering_pwm) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorsOrder_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorsOrder_

// alias to use template instance with default allocator
using MotorsOrder =
  interfaces::msg::MotorsOrder_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__MOTORS_ORDER__STRUCT_HPP_
