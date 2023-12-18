// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_H_
#define PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SpeedControl in the package pappap.
typedef struct pappap__srv__SpeedControl_Request
{
  float spd;
} pappap__srv__SpeedControl_Request;

// Struct for a sequence of pappap__srv__SpeedControl_Request.
typedef struct pappap__srv__SpeedControl_Request__Sequence
{
  pappap__srv__SpeedControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__srv__SpeedControl_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'feedback'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SpeedControl in the package pappap.
typedef struct pappap__srv__SpeedControl_Response
{
  rosidl_runtime_c__String feedback;
} pappap__srv__SpeedControl_Response;

// Struct for a sequence of pappap__srv__SpeedControl_Response.
typedef struct pappap__srv__SpeedControl_Response__Sequence
{
  pappap__srv__SpeedControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} pappap__srv__SpeedControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PAPPAP__SRV__DETAIL__SPEED_CONTROL__STRUCT_H_
