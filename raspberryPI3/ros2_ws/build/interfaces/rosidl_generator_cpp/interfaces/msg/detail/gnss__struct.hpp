// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/Gnss.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__Gnss __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__Gnss __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Gnss_
{
  using Type = Gnss_<ContainerAllocator>;

  explicit Gnss_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->latitude = -1.0;
      this->longitude = -1.0;
      this->altitude = -1.0;
      this->quality = -1;
      this->hacc = -1.0;
      this->vacc = -1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->quality = 0;
      this->hacc = 0.0;
      this->vacc = 0.0;
    }
  }

  explicit Gnss_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->latitude = -1.0;
      this->longitude = -1.0;
      this->altitude = -1.0;
      this->quality = -1;
      this->hacc = -1.0;
      this->vacc = -1.0;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->quality = 0;
      this->hacc = 0.0;
      this->vacc = 0.0;
    }
  }

  // field types and members
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _quality_type =
    int8_t;
  _quality_type quality;
  using _hacc_type =
    double;
  _hacc_type hacc;
  using _vacc_type =
    double;
  _vacc_type vacc;

  // setters for named parameter idiom
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__quality(
    const int8_t & _arg)
  {
    this->quality = _arg;
    return *this;
  }
  Type & set__hacc(
    const double & _arg)
  {
    this->hacc = _arg;
    return *this;
  }
  Type & set__vacc(
    const double & _arg)
  {
    this->vacc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::Gnss_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::Gnss_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::Gnss_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::Gnss_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Gnss_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Gnss_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::Gnss_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::Gnss_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::Gnss_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::Gnss_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__Gnss
    std::shared_ptr<interfaces::msg::Gnss_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__Gnss
    std::shared_ptr<interfaces::msg::Gnss_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Gnss_ & other) const
  {
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->quality != other.quality) {
      return false;
    }
    if (this->hacc != other.hacc) {
      return false;
    }
    if (this->vacc != other.vacc) {
      return false;
    }
    return true;
  }
  bool operator!=(const Gnss_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Gnss_

// alias to use template instance with default allocator
using Gnss =
  interfaces::msg::Gnss_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__GNSS__STRUCT_HPP_
