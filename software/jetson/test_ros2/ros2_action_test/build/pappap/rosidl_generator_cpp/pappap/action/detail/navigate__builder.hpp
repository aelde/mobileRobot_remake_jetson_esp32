// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pappap:action/Navigate.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_
#define PAPPAP__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pappap/action/detail/navigate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_Goal_target
{
public:
  Init_Navigate_Goal_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Navigate_Goal target(::pappap::action::Navigate_Goal::_target_type arg)
  {
    msg_.target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_Goal>()
{
  return pappap::action::builder::Init_Navigate_Goal_target();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_Result_used_time
{
public:
  Init_Navigate_Result_used_time()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Navigate_Result used_time(::pappap::action::Navigate_Result::_used_time_type arg)
  {
    msg_.used_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_Result>()
{
  return pappap::action::builder::Init_Navigate_Result_used_time();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_Feedback_distance_to_target
{
public:
  Init_Navigate_Feedback_distance_to_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Navigate_Feedback distance_to_target(::pappap::action::Navigate_Feedback::_distance_to_target_type arg)
  {
    msg_.distance_to_target = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_Feedback>()
{
  return pappap::action::builder::Init_Navigate_Feedback_distance_to_target();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_SendGoal_Request_goal
{
public:
  explicit Init_Navigate_SendGoal_Request_goal(::pappap::action::Navigate_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::pappap::action::Navigate_SendGoal_Request goal(::pappap::action::Navigate_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_SendGoal_Request msg_;
};

class Init_Navigate_SendGoal_Request_goal_id
{
public:
  Init_Navigate_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_SendGoal_Request_goal goal_id(::pappap::action::Navigate_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigate_SendGoal_Request_goal(msg_);
  }

private:
  ::pappap::action::Navigate_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_SendGoal_Request>()
{
  return pappap::action::builder::Init_Navigate_SendGoal_Request_goal_id();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_SendGoal_Response_stamp
{
public:
  explicit Init_Navigate_SendGoal_Response_stamp(::pappap::action::Navigate_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::pappap::action::Navigate_SendGoal_Response stamp(::pappap::action::Navigate_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_SendGoal_Response msg_;
};

class Init_Navigate_SendGoal_Response_accepted
{
public:
  Init_Navigate_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_SendGoal_Response_stamp accepted(::pappap::action::Navigate_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Navigate_SendGoal_Response_stamp(msg_);
  }

private:
  ::pappap::action::Navigate_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_SendGoal_Response>()
{
  return pappap::action::builder::Init_Navigate_SendGoal_Response_accepted();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_GetResult_Request_goal_id
{
public:
  Init_Navigate_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Navigate_GetResult_Request goal_id(::pappap::action::Navigate_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_GetResult_Request>()
{
  return pappap::action::builder::Init_Navigate_GetResult_Request_goal_id();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_GetResult_Response_result
{
public:
  explicit Init_Navigate_GetResult_Response_result(::pappap::action::Navigate_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::pappap::action::Navigate_GetResult_Response result(::pappap::action::Navigate_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_GetResult_Response msg_;
};

class Init_Navigate_GetResult_Response_status
{
public:
  Init_Navigate_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_GetResult_Response_result status(::pappap::action::Navigate_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Navigate_GetResult_Response_result(msg_);
  }

private:
  ::pappap::action::Navigate_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_GetResult_Response>()
{
  return pappap::action::builder::Init_Navigate_GetResult_Response_status();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Navigate_FeedbackMessage_feedback
{
public:
  explicit Init_Navigate_FeedbackMessage_feedback(::pappap::action::Navigate_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::pappap::action::Navigate_FeedbackMessage feedback(::pappap::action::Navigate_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Navigate_FeedbackMessage msg_;
};

class Init_Navigate_FeedbackMessage_goal_id
{
public:
  Init_Navigate_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_FeedbackMessage_feedback goal_id(::pappap::action::Navigate_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Navigate_FeedbackMessage_feedback(msg_);
  }

private:
  ::pappap::action::Navigate_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Navigate_FeedbackMessage>()
{
  return pappap::action::builder::Init_Navigate_FeedbackMessage_goal_id();
}

}  // namespace pappap

#endif  // PAPPAP__ACTION__DETAIL__NAVIGATE__BUILDER_HPP_
