// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice
#include "pappap/srv/detail/speed_control__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "pappap/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pappap/srv/detail/speed_control__struct.h"
#include "pappap/srv/detail/speed_control__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _SpeedControl_Request__ros_msg_type = pappap__srv__SpeedControl_Request;

static bool _SpeedControl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SpeedControl_Request__ros_msg_type * ros_message = static_cast<const _SpeedControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: spd
  {
    cdr << ros_message->spd;
  }

  return true;
}

static bool _SpeedControl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SpeedControl_Request__ros_msg_type * ros_message = static_cast<_SpeedControl_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: spd
  {
    cdr >> ros_message->spd;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pappap
size_t get_serialized_size_pappap__srv__SpeedControl_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpeedControl_Request__ros_msg_type * ros_message = static_cast<const _SpeedControl_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name spd
  {
    size_t item_size = sizeof(ros_message->spd);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SpeedControl_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pappap__srv__SpeedControl_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pappap
size_t max_serialized_size_pappap__srv__SpeedControl_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: spd
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _SpeedControl_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pappap__srv__SpeedControl_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SpeedControl_Request = {
  "pappap::srv",
  "SpeedControl_Request",
  _SpeedControl_Request__cdr_serialize,
  _SpeedControl_Request__cdr_deserialize,
  _SpeedControl_Request__get_serialized_size,
  _SpeedControl_Request__max_serialized_size
};

static rosidl_message_type_support_t _SpeedControl_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpeedControl_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pappap, srv, SpeedControl_Request)() {
  return &_SpeedControl_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "pappap/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "pappap/srv/detail/speed_control__struct.h"
// already included above
// #include "pappap/srv/detail/speed_control__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // feedback
#include "rosidl_runtime_c/string_functions.h"  // feedback

// forward declare type support functions


using _SpeedControl_Response__ros_msg_type = pappap__srv__SpeedControl_Response;

static bool _SpeedControl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SpeedControl_Response__ros_msg_type * ros_message = static_cast<const _SpeedControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: feedback
  {
    const rosidl_runtime_c__String * str = &ros_message->feedback;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _SpeedControl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SpeedControl_Response__ros_msg_type * ros_message = static_cast<_SpeedControl_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: feedback
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->feedback.data) {
      rosidl_runtime_c__String__init(&ros_message->feedback);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->feedback,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'feedback'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pappap
size_t get_serialized_size_pappap__srv__SpeedControl_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpeedControl_Response__ros_msg_type * ros_message = static_cast<const _SpeedControl_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name feedback
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->feedback.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _SpeedControl_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_pappap__srv__SpeedControl_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_pappap
size_t max_serialized_size_pappap__srv__SpeedControl_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: feedback
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _SpeedControl_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_pappap__srv__SpeedControl_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SpeedControl_Response = {
  "pappap::srv",
  "SpeedControl_Response",
  _SpeedControl_Response__cdr_serialize,
  _SpeedControl_Response__cdr_deserialize,
  _SpeedControl_Response__get_serialized_size,
  _SpeedControl_Response__max_serialized_size
};

static rosidl_message_type_support_t _SpeedControl_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpeedControl_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pappap, srv, SpeedControl_Response)() {
  return &_SpeedControl_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "pappap/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "pappap/srv/speed_control.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t SpeedControl__callbacks = {
  "pappap::srv",
  "SpeedControl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pappap, srv, SpeedControl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pappap, srv, SpeedControl_Response)(),
};

static rosidl_service_type_support_t SpeedControl__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &SpeedControl__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, pappap, srv, SpeedControl)() {
  return &SpeedControl__handle;
}

#if defined(__cplusplus)
}
#endif
