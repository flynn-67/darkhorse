// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from limo_interfaces:action/Speak.idl
// generated code does not contain a copyright notice

#ifndef LIMO_INTERFACES__ACTION__DETAIL__SPEAK__STRUCT_H_
#define LIMO_INTERFACES__ACTION__DETAIL__SPEAK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'text'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_Goal
{
  rosidl_runtime_c__String text;
} limo_interfaces__action__Speak_Goal;

// Struct for a sequence of limo_interfaces__action__Speak_Goal.
typedef struct limo_interfaces__action__Speak_Goal__Sequence
{
  limo_interfaces__action__Speak_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_Result
{
  bool success;
} limo_interfaces__action__Speak_Result;

// Struct for a sequence of limo_interfaces__action__Speak_Result.
typedef struct limo_interfaces__action__Speak_Result__Sequence
{
  limo_interfaces__action__Speak_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_Feedback
{
  float progress;
} limo_interfaces__action__Speak_Feedback;

// Struct for a sequence of limo_interfaces__action__Speak_Feedback.
typedef struct limo_interfaces__action__Speak_Feedback__Sequence
{
  limo_interfaces__action__Speak_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "limo_interfaces/action/detail/speak__struct.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  limo_interfaces__action__Speak_Goal goal;
} limo_interfaces__action__Speak_SendGoal_Request;

// Struct for a sequence of limo_interfaces__action__Speak_SendGoal_Request.
typedef struct limo_interfaces__action__Speak_SendGoal_Request__Sequence
{
  limo_interfaces__action__Speak_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} limo_interfaces__action__Speak_SendGoal_Response;

// Struct for a sequence of limo_interfaces__action__Speak_SendGoal_Response.
typedef struct limo_interfaces__action__Speak_SendGoal_Response__Sequence
{
  limo_interfaces__action__Speak_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} limo_interfaces__action__Speak_GetResult_Request;

// Struct for a sequence of limo_interfaces__action__Speak_GetResult_Request.
typedef struct limo_interfaces__action__Speak_GetResult_Request__Sequence
{
  limo_interfaces__action__Speak_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_GetResult_Response
{
  int8_t status;
  limo_interfaces__action__Speak_Result result;
} limo_interfaces__action__Speak_GetResult_Response;

// Struct for a sequence of limo_interfaces__action__Speak_GetResult_Response.
typedef struct limo_interfaces__action__Speak_GetResult_Response__Sequence
{
  limo_interfaces__action__Speak_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "limo_interfaces/action/detail/speak__struct.h"

/// Struct defined in action/Speak in the package limo_interfaces.
typedef struct limo_interfaces__action__Speak_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  limo_interfaces__action__Speak_Feedback feedback;
} limo_interfaces__action__Speak_FeedbackMessage;

// Struct for a sequence of limo_interfaces__action__Speak_FeedbackMessage.
typedef struct limo_interfaces__action__Speak_FeedbackMessage__Sequence
{
  limo_interfaces__action__Speak_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} limo_interfaces__action__Speak_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIMO_INTERFACES__ACTION__DETAIL__SPEAK__STRUCT_H_
