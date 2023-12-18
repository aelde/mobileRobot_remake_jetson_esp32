// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_HPP_
#define PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__pappap__srv__SpeedControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__pappap__srv__SpeedControl_Request __declspec(deprecated)
#endif

namespace pappap
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SpeedControl_Request_
{
  using Type = SpeedControl_Request_<ContainerAllocator>;

  explicit SpeedControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->spd = 0.0f;
    }
  }

  explicit SpeedControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->spd = 0.0f;
    }
  }

  // field types and members
  using _spd_type =
    float;
  _spd_type spd;

  // setters for named parameter idiom
  Type & set__spd(
    const float & _arg)
  {
    this->spd = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pappap::srv::SpeedControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const pappap::srv::SpeedControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pappap::srv::SpeedControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pappap::srv::SpeedControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pappap__srv__SpeedControl_Request
    std::shared_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pappap__srv__SpeedControl_Request
    std::shared_ptr<pappap::srv::SpeedControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeedControl_Request_ & other) const
  {
    if (this->spd != other.spd) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeedControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeedControl_Request_

// alias to use template instance with default allocator
using SpeedControl_Request =
  pappap::srv::SpeedControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pappap


#ifndef _WIN32
# define DEPRECATED__pappap__srv__SpeedControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__pappap__srv__SpeedControl_Response __declspec(deprecated)
#endif

namespace pappap
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SpeedControl_Response_
{
  using Type = SpeedControl_Response_<ContainerAllocator>;

  explicit SpeedControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = "";
    }
  }

  explicit SpeedControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : feedback(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->feedback = "";
    }
  }

  // field types and members
  using _feedback_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__feedback(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    pappap::srv::SpeedControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const pappap::srv::SpeedControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      pappap::srv::SpeedControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      pappap::srv::SpeedControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__pappap__srv__SpeedControl_Response
    std::shared_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__pappap__srv__SpeedControl_Response
    std::shared_ptr<pappap::srv::SpeedControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeedControl_Response_ & other) const
  {
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeedControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeedControl_Response_

// alias to use template instance with default allocator
using SpeedControl_Response =
  pappap::srv::SpeedControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace pappap

namespace pappap
{

namespace srv
{

struct SpeedControl
{
  using Request = pappap::srv::SpeedControl_Request;
  using Response = pappap::srv::SpeedControl_Response;
};

}  // namespace srv

}  // namespace pappap

#endif  // PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_HPP_
