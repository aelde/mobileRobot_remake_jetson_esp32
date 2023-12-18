// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__SRV__DETAIL__SPEED_CONTROL__BUILDER_HPP_
#define PAPPAP__SRV__DETAIL__SPEED_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pappap/srv/detail/speed_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pappap
{

namespace srv
{

namespace builder
{

class Init_SpeedControl_Request_spd
{
public:
  Init_SpeedControl_Request_spd()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::srv::SpeedControl_Request spd(::pappap::srv::SpeedControl_Request::_spd_type arg)
  {
    msg_.spd = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::srv::SpeedControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::srv::SpeedControl_Request>()
{
  return pappap::srv::builder::Init_SpeedControl_Request_spd();
}

}  // namespace pappap


namespace pappap
{

namespace srv
{

namespace builder
{

class Init_SpeedControl_Response_feedback
{
public:
  Init_SpeedControl_Response_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::srv::SpeedControl_Response feedback(::pappap::srv::SpeedControl_Response::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::srv::SpeedControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::srv::SpeedControl_Response>()
{
  return pappap::srv::builder::Init_SpeedControl_Response_feedback();
}

}  // namespace pappap

#endif  // PAPPAP__SRV__DETAIL__SPEED_CONTROL__BUILDER_HPP_
