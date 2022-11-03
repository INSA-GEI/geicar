// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/SteeringCalibration.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__SteeringCalibration __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__SteeringCalibration __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SteeringCalibration_
{
  using Type = SteeringCalibration_<ContainerAllocator>;

  explicit SteeringCalibration_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = false;
      this->in_progress = false;
      this->user_need = false;
      this->status = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
      this->in_progress = false;
      this->user_need = false;
      this->status = 0;
    }
  }

  explicit SteeringCalibration_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = false;
      this->in_progress = false;
      this->user_need = false;
      this->status = 0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
      this->in_progress = false;
      this->user_need = false;
      this->status = 0;
    }
  }

  // field types and members
  using _request_type =
    bool;
  _request_type request;
  using _in_progress_type =
    bool;
  _in_progress_type in_progress;
  using _user_need_type =
    bool;
  _user_need_type user_need;
  using _status_type =
    int8_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__request(
    const bool & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__in_progress(
    const bool & _arg)
  {
    this->in_progress = _arg;
    return *this;
  }
  Type & set__user_need(
    const bool & _arg)
  {
    this->user_need = _arg;
    return *this;
  }
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::SteeringCalibration_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::SteeringCalibration_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::SteeringCalibration_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::SteeringCalibration_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__SteeringCalibration
    std::shared_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__SteeringCalibration
    std::shared_ptr<interfaces::msg::SteeringCalibration_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SteeringCalibration_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    if (this->in_progress != other.in_progress) {
      return false;
    }
    if (this->user_need != other.user_need) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const SteeringCalibration_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SteeringCalibration_

// alias to use template instance with default allocator
using SteeringCalibration =
  interfaces::msg::SteeringCalibration_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__STEERING_CALIBRATION__STRUCT_HPP_
