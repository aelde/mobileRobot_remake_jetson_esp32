// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pappap:action/Test.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__ACTION__DETAIL__TEST__STRUCT_H_
#define PAPPAP__ACTION__DETAIL__TEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_Goal
{
  int32_t secs;
} pappap__action__Test_Goal;

// Struct for a sequence of pappap__action__Test_Goal.
typedef struct pappap__action__Test_Goal__Sequence
{
  pappap__action__Test_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_Goal__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_Result
{
  rosidl_runtime_c__String status;
} pappap__action__Test_Result;

// Struct for a sequence of pappap__action__Test_Result.
typedef struct pappap__action__Test_Result__Sequence
{
  pappap__action__Test_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_Result__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_Feedback
{
  rosidl_runtime_c__String feedback;
} pappap__action__Test_Feedback;

// Struct for a sequence of pappap__action__Test_Feedback.
typedef struct pappap__action__Test_Feedback__Sequence
{
  pappap__action__Test_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "pappap/action/detail/test__struct.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  pappap__action__Test_Goal goal;
} pappap__action__Test_SendGoal_Request;

// Struct for a sequence of pappap__action__Test_SendGoal_Request.
typedef struct pappap__action__Test_SendGoal_Request__Sequence
{
  pappap__action__Test_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} pappap__action__Test_SendGoal_Response;

// Struct for a sequence of pappap__action__Test_SendGoal_Response.
typedef struct pappap__action__Test_SendGoal_Response__Sequence
{
  pappap__action__Test_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} pappap__action__Test_GetResult_Request;

// Struct for a sequence of pappap__action__Test_GetResult_Request.
typedef struct pappap__action__Test_GetResult_Request__Sequence
{
  pappap__action__Test_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "pappap/action/detail/test__struct.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_GetResult_Response
{
  int8_t status;
  pappap__action__Test_Result result;
} pappap__action__Test_GetResult_Response;

// Struct for a sequence of pappap__action__Test_GetResult_Response.
typedef struct pappap__action__Test_GetResult_Response__Sequence
{
  pappap__action__Test_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "pappap/action/detail/test__struct.h"

/// Struct defined in action/Test in the package pappap.
typedef struct pappap__action__Test_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  pappap__action__Test_Feedback feedback;
} pappap__action__Test_FeedbackMessage;

// Struct for a sequence of pappap__action__Test_FeedbackMessage.
typedef struct pappap__action__Test_FeedbackMessage__Sequence
{
  pappap__action__Test_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__action__Test_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PAPPAP__ACTION__DETAIL__TEST__STRUCT_H_
