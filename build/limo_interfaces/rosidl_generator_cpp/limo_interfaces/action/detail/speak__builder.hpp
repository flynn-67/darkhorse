// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from limo_interfaces:action/Speak.idl
// generated code does not contain a copyright notice

#ifndef LIMO_INTERFACES__ACTION__DETAIL__SPEAK__BUILDER_HPP_
#define LIMO_INTERFACES__ACTION__DETAIL__SPEAK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "limo_interfaces/action/detail/speak__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_Goal_text
{
public:
  Init_Speak_Goal_text()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::limo_interfaces::action::Speak_Goal text(::limo_interfaces::action::Speak_Goal::_text_type arg)
  {
    msg_.text = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_Goal>()
{
  return limo_interfaces::action::builder::Init_Speak_Goal_text();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_Result_success
{
public:
  Init_Speak_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::limo_interfaces::action::Speak_Result success(::limo_interfaces::action::Speak_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_Result>()
{
  return limo_interfaces::action::builder::Init_Speak_Result_success();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_Feedback_progress
{
public:
  Init_Speak_Feedback_progress()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::limo_interfaces::action::Speak_Feedback progress(::limo_interfaces::action::Speak_Feedback::_progress_type arg)
  {
    msg_.progress = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_Feedback>()
{
  return limo_interfaces::action::builder::Init_Speak_Feedback_progress();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_SendGoal_Request_goal
{
public:
  explicit Init_Speak_SendGoal_Request_goal(::limo_interfaces::action::Speak_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::limo_interfaces::action::Speak_SendGoal_Request goal(::limo_interfaces::action::Speak_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_SendGoal_Request msg_;
};

class Init_Speak_SendGoal_Request_goal_id
{
public:
  Init_Speak_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Speak_SendGoal_Request_goal goal_id(::limo_interfaces::action::Speak_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Speak_SendGoal_Request_goal(msg_);
  }

private:
  ::limo_interfaces::action::Speak_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_SendGoal_Request>()
{
  return limo_interfaces::action::builder::Init_Speak_SendGoal_Request_goal_id();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_SendGoal_Response_stamp
{
public:
  explicit Init_Speak_SendGoal_Response_stamp(::limo_interfaces::action::Speak_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::limo_interfaces::action::Speak_SendGoal_Response stamp(::limo_interfaces::action::Speak_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_SendGoal_Response msg_;
};

class Init_Speak_SendGoal_Response_accepted
{
public:
  Init_Speak_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Speak_SendGoal_Response_stamp accepted(::limo_interfaces::action::Speak_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Speak_SendGoal_Response_stamp(msg_);
  }

private:
  ::limo_interfaces::action::Speak_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_SendGoal_Response>()
{
  return limo_interfaces::action::builder::Init_Speak_SendGoal_Response_accepted();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_GetResult_Request_goal_id
{
public:
  Init_Speak_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::limo_interfaces::action::Speak_GetResult_Request goal_id(::limo_interfaces::action::Speak_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_GetResult_Request>()
{
  return limo_interfaces::action::builder::Init_Speak_GetResult_Request_goal_id();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_GetResult_Response_result
{
public:
  explicit Init_Speak_GetResult_Response_result(::limo_interfaces::action::Speak_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::limo_interfaces::action::Speak_GetResult_Response result(::limo_interfaces::action::Speak_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_GetResult_Response msg_;
};

class Init_Speak_GetResult_Response_status
{
public:
  Init_Speak_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Speak_GetResult_Response_result status(::limo_interfaces::action::Speak_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Speak_GetResult_Response_result(msg_);
  }

private:
  ::limo_interfaces::action::Speak_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_GetResult_Response>()
{
  return limo_interfaces::action::builder::Init_Speak_GetResult_Response_status();
}

}  // namespace limo_interfaces


namespace limo_interfaces
{

namespace action
{

namespace builder
{

class Init_Speak_FeedbackMessage_feedback
{
public:
  explicit Init_Speak_FeedbackMessage_feedback(::limo_interfaces::action::Speak_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::limo_interfaces::action::Speak_FeedbackMessage feedback(::limo_interfaces::action::Speak_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::limo_interfaces::action::Speak_FeedbackMessage msg_;
};

class Init_Speak_FeedbackMessage_goal_id
{
public:
  Init_Speak_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Speak_FeedbackMessage_feedback goal_id(::limo_interfaces::action::Speak_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Speak_FeedbackMessage_feedback(msg_);
  }

private:
  ::limo_interfaces::action::Speak_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::limo_interfaces::action::Speak_FeedbackMessage>()
{
  return limo_interfaces::action::builder::Init_Speak_FeedbackMessage_goal_id();
}

}  // namespace limo_interfaces

#endif  // LIMO_INTERFACES__ACTION__DETAIL__SPEAK__BUILDER_HPP_
