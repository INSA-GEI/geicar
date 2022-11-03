// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:msg/SystemCheck.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_HPP_
#define INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__msg__SystemCheck __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__msg__SystemCheck __declspec(deprecated)
#endif

namespace interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SystemCheck_
{
  using Type = SystemCheck_<ContainerAllocator>;

  explicit SystemCheck_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = false;
      this->response = false;
      this->report = false;
      this->print = false;
      this->jetson = false;
      this->l476 = false;
      this->f103 = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
      this->response = false;
      this->report = false;
      this->print = false;
      this->jetson = false;
      this->l476 = false;
      this->f103 = false;
      this->comm_jetson = "";
      this->comm_l476 = "";
      this->comm_f103 = "";
      this->battery = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ultrasonics.begin(), this->ultrasonics.end(), "");
      this->gps = "";
      this->imu = "";
      this->lidar = "";
      this->camera = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->comm_jetson = "";
      this->comm_l476 = "";
      this->comm_f103 = "";
      this->battery = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ultrasonics.begin(), this->ultrasonics.end(), "");
      this->gps = "";
      this->imu = "";
      this->lidar = "";
      this->camera = "";
    }
  }

  explicit SystemCheck_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : comm_jetson(_alloc),
    comm_l476(_alloc),
    comm_f103(_alloc),
    battery(_alloc),
    ultrasonics(_alloc),
    gps(_alloc),
    imu(_alloc),
    lidar(_alloc),
    camera(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->request = false;
      this->response = false;
      this->report = false;
      this->print = false;
      this->jetson = false;
      this->l476 = false;
      this->f103 = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->request = false;
      this->response = false;
      this->report = false;
      this->print = false;
      this->jetson = false;
      this->l476 = false;
      this->f103 = false;
      this->comm_jetson = "";
      this->comm_l476 = "";
      this->comm_f103 = "";
      this->battery = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ultrasonics.begin(), this->ultrasonics.end(), "");
      this->gps = "";
      this->imu = "";
      this->lidar = "";
      this->camera = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->comm_jetson = "";
      this->comm_l476 = "";
      this->comm_f103 = "";
      this->battery = "";
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->ultrasonics.begin(), this->ultrasonics.end(), "");
      this->gps = "";
      this->imu = "";
      this->lidar = "";
      this->camera = "";
    }
  }

  // field types and members
  using _request_type =
    bool;
  _request_type request;
  using _response_type =
    bool;
  _response_type response;
  using _report_type =
    bool;
  _report_type report;
  using _print_type =
    bool;
  _print_type print;
  using _jetson_type =
    bool;
  _jetson_type jetson;
  using _l476_type =
    bool;
  _l476_type l476;
  using _f103_type =
    bool;
  _f103_type f103;
  using _comm_jetson_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _comm_jetson_type comm_jetson;
  using _comm_l476_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _comm_l476_type comm_l476;
  using _comm_f103_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _comm_f103_type comm_f103;
  using _battery_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _battery_type battery;
  using _ultrasonics_type =
    std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6>;
  _ultrasonics_type ultrasonics;
  using _gps_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gps_type gps;
  using _imu_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _imu_type imu;
  using _lidar_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _lidar_type lidar;
  using _camera_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _camera_type camera;

  // setters for named parameter idiom
  Type & set__request(
    const bool & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const bool & _arg)
  {
    this->response = _arg;
    return *this;
  }
  Type & set__report(
    const bool & _arg)
  {
    this->report = _arg;
    return *this;
  }
  Type & set__print(
    const bool & _arg)
  {
    this->print = _arg;
    return *this;
  }
  Type & set__jetson(
    const bool & _arg)
  {
    this->jetson = _arg;
    return *this;
  }
  Type & set__l476(
    const bool & _arg)
  {
    this->l476 = _arg;
    return *this;
  }
  Type & set__f103(
    const bool & _arg)
  {
    this->f103 = _arg;
    return *this;
  }
  Type & set__comm_jetson(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->comm_jetson = _arg;
    return *this;
  }
  Type & set__comm_l476(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->comm_l476 = _arg;
    return *this;
  }
  Type & set__comm_f103(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->comm_f103 = _arg;
    return *this;
  }
  Type & set__battery(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->battery = _arg;
    return *this;
  }
  Type & set__ultrasonics(
    const std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 6> & _arg)
  {
    this->ultrasonics = _arg;
    return *this;
  }
  Type & set__gps(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gps = _arg;
    return *this;
  }
  Type & set__imu(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->imu = _arg;
    return *this;
  }
  Type & set__lidar(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->lidar = _arg;
    return *this;
  }
  Type & set__camera(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->camera = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::msg::SystemCheck_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::msg::SystemCheck_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::msg::SystemCheck_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::msg::SystemCheck_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::SystemCheck_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::SystemCheck_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::msg::SystemCheck_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::msg::SystemCheck_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::msg::SystemCheck_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::msg::SystemCheck_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__msg__SystemCheck
    std::shared_ptr<interfaces::msg::SystemCheck_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__msg__SystemCheck
    std::shared_ptr<interfaces::msg::SystemCheck_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SystemCheck_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    if (this->report != other.report) {
      return false;
    }
    if (this->print != other.print) {
      return false;
    }
    if (this->jetson != other.jetson) {
      return false;
    }
    if (this->l476 != other.l476) {
      return false;
    }
    if (this->f103 != other.f103) {
      return false;
    }
    if (this->comm_jetson != other.comm_jetson) {
      return false;
    }
    if (this->comm_l476 != other.comm_l476) {
      return false;
    }
    if (this->comm_f103 != other.comm_f103) {
      return false;
    }
    if (this->battery != other.battery) {
      return false;
    }
    if (this->ultrasonics != other.ultrasonics) {
      return false;
    }
    if (this->gps != other.gps) {
      return false;
    }
    if (this->imu != other.imu) {
      return false;
    }
    if (this->lidar != other.lidar) {
      return false;
    }
    if (this->camera != other.camera) {
      return false;
    }
    return true;
  }
  bool operator!=(const SystemCheck_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SystemCheck_

// alias to use template instance with default allocator
using SystemCheck =
  interfaces::msg::SystemCheck_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__SYSTEM_CHECK__STRUCT_HPP_
