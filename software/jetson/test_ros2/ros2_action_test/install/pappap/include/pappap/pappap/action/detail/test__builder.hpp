// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from pappap:action/Test.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__ACTION__DETAIL__TEST__BUILDER_HPP_
#define PAPPAP__ACTION__DETAIL__TEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "pappap/action/detail/test__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_Goal_secs
{
public:
  Init_Test_Goal_secs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Test_Goal secs(::pappap::action::Test_Goal::_secs_type arg)
  {
    msg_.secs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_Goal>()
{
  return pappap::action::builder::Init_Test_Goal_secs();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_Result_status
{
public:
  Init_Test_Result_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Test_Result status(::pappap::action::Test_Result::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_Result>()
{
  return pappap::action::builder::Init_Test_Result_status();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_Feedback_feedback
{
public:
  Init_Test_Feedback_feedback()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Test_Feedback feedback(::pappap::action::Test_Feedback::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_Feedback>()
{
  return pappap::action::builder::Init_Test_Feedback_feedback();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_SendGoal_Request_goal
{
public:
  explicit Init_Test_SendGoal_Request_goal(::pappap::action::Test_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::pappap::action::Test_SendGoal_Request goal(::pappap::action::Test_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_SendGoal_Request msg_;
};

class Init_Test_SendGoal_Request_goal_id
{
public:
  Init_Test_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_SendGoal_Request_goal goal_id(::pappap::action::Test_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Test_SendGoal_Request_goal(msg_);
  }

private:
  ::pappap::action::Test_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_SendGoal_Request>()
{
  return pappap::action::builder::Init_Test_SendGoal_Request_goal_id();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_SendGoal_Response_stamp
{
public:
  explicit Init_Test_SendGoal_Response_stamp(::pappap::action::Test_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::pappap::action::Test_SendGoal_Response stamp(::pappap::action::Test_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_SendGoal_Response msg_;
};

class Init_Test_SendGoal_Response_accepted
{
public:
  Init_Test_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_SendGoal_Response_stamp accepted(::pappap::action::Test_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Test_SendGoal_Response_stamp(msg_);
  }

private:
  ::pappap::action::Test_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_SendGoal_Response>()
{
  return pappap::action::builder::Init_Test_SendGoal_Response_accepted();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_GetResult_Request_goal_id
{
public:
  Init_Test_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::pappap::action::Test_GetResult_Request goal_id(::pappap::action::Test_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_GetResult_Request>()
{
  return pappap::action::builder::Init_Test_GetResult_Request_goal_id();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_GetResult_Response_result
{
public:
  explicit Init_Test_GetResult_Response_result(::pappap::action::Test_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::pappap::action::Test_GetResult_Response result(::pappap::action::Test_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_GetResult_Response msg_;
};

class Init_Test_GetResult_Response_status
{
public:
  Init_Test_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_GetResult_Response_result status(::pappap::action::Test_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Test_GetResult_Response_result(msg_);
  }

private:
  ::pappap::action::Test_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_GetResult_Response>()
{
  return pappap::action::builder::Init_Test_GetResult_Response_status();
}

}  // namespace pappap


namespace pappap
{

namespace action
{

namespace builder
{

class Init_Test_FeedbackMessage_feedback
{
public:
  explicit Init_Test_FeedbackMessage_feedback(::pappap::action::Test_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::pappap::action::Test_FeedbackMessage feedback(::pappap::action::Test_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::pappap::action::Test_FeedbackMessage msg_;
};

class Init_Test_FeedbackMessage_goal_id
{
public:
  Init_Test_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Test_FeedbackMessage_feedback goal_id(::pappap::action::Test_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Test_FeedbackMessage_feedback(msg_);
  }

private:
  ::pappap::action::Test_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::pappap::action::Test_FeedbackMessage>()
{
  return pappap::action::builder::Init_Test_FeedbackMessage_goal_id();
}

}  // namespace pappap

#endif  // PAPPAP__ACTION__DETAIL__TEST__BUILDER_HPP_
