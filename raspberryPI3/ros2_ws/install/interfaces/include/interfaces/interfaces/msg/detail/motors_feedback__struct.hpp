// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/MotorsFeedback.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__MotorsFeedback __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__MotorsFeedback __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorsFeedback_
{
  using Type = MotorsFeedback_<ContainerAllocator>;

  explicit MotorsFeedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->left_rear_odometry = 0;
      this->right_rear_odometry = 0;
      this->left_rear_speed = 0.0f;
      this->right_rear_speed = 0.0f;
      this->steering_angle = 0.0f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->left_rear_odometry = 0;
      this->right_rear_odometry = 0;
      this->left_rear_speed = 0.0f;
      this->right_rear_speed = 0.0f;
      this->steering_angle = 0.0f;
    }
  }

  explicit MotorsFeedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->left_rear_odometry = 0;
      this->right_rear_odometry = 0;
      this->left_rear_speed = 0.0f;
      this->right_rear_speed = 0.0f;
      this->steering_angle = 0.0f;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->left_rear_odometry = 0;
      this->right_rear_odometry = 0;
      this->left_rear_speed = 0.0f;
      this->right_rear_speed = 0.0f;
      this->steering_angle = 0.0f;
    }
  }

  // field types and members
  using _left_rear_odometry_type =
    int8_t;
  _left_rear_odometry_type left_rear_odometry;
  using _right_rear_odometry_type =
    int8_t;
  _right_rear_odometry_type right_rear_odometry;
  using _left_rear_speed_type =
    float;
  _left_rear_speed_type left_rear_speed;
  using _right_rear_speed_type =
    float;
  _right_rear_speed_type right_rear_speed;
  using _steering_angle_type =
    float;
  _steering_angle_type steering_angle;

  // setters for named parameter idiom
  Type & set__left_rear_odometry(
    const int8_t & _arg)
  {
    this->left_rear_odometry = _arg;
    return *this;
  }
  Type & set__right_rear_odometry(
    const int8_t & _arg)
  {
    this->right_rear_odometry = _arg;
    return *this;
  }
  Type & set__left_rear_speed(
    const float & _arg)
  {
    this->left_rear_speed = _arg;
    return *this;
  }
  Type & set__right_rear_speed(
    const float & _arg)
  {
    this->right_rear_speed = _arg;
    return *this;
  }
  Type & set__steering_angle(
    const float & _arg)
  {
    this->steering_angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::MotorsFeedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::MotorsFeedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::MotorsFeedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::MotorsFeedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__MotorsFeedback
    std::shared_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__MotorsFeedback
    std::shared_ptr<interfaces::msg::MotorsFeedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorsFeedback_ & other) const
  {
    if (this->left_rear_odometry != other.left_rear_odometry) {
      return false;
    }
    if (this->right_rear_odometry != other.right_rear_odometry) {
      return false;
    }
    if (this->left_rear_speed != other.left_rear_speed) {
      return false;
    }
    if (this->right_rear_speed != other.right_rear_speed) {
      return false;
    }
    if (this->steering_angle != other.steering_angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorsFeedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorsFeedback_

// alias to use template instance with default allocator
using MotorsFeedback =
  interfaces::msg::MotorsFeedback_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__MOTORS_FEEDBACK__STRUCT_HPP_
