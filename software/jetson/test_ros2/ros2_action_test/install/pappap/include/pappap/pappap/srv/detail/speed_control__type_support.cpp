// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "pappap/srv/detail/speed_control__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pappap
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SpeedControl_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pappap::srv::SpeedControl_Request(_init);
}

void SpeedControl_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pappap::srv::SpeedControl_Request *>(message_memory);
  typed_message->~SpeedControl_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SpeedControl_Request_message_member_array[1] = {
  {
    "spd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pappap::srv::SpeedControl_Request, spd),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SpeedControl_Request_message_members = {
  "pappap::srv",  // message namespace
  "SpeedControl_Request",  // message name
  1,  // number of fields
  sizeof(pappap::srv::SpeedControl_Request),
  SpeedControl_Request_message_member_array,  // message members
  SpeedControl_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeedControl_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SpeedControl_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeedControl_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pappap


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pappap::srv::SpeedControl_Request>()
{
  return &::pappap::srv::rosidl_typesupport_introspection_cpp::SpeedControl_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pappap, srv, SpeedControl_Request)() {
  return &::pappap::srv::rosidl_typesupport_introspection_cpp::SpeedControl_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "pappap/srv/detail/speed_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace pappap
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void SpeedControl_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) pappap::srv::SpeedControl_Response(_init);
}

void SpeedControl_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<pappap::srv::SpeedControl_Response *>(message_memory);
  typed_message->~SpeedControl_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SpeedControl_Response_message_member_array[1] = {
  {
    "feedback",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(pappap::srv::SpeedControl_Response, feedback),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SpeedControl_Response_message_members = {
  "pappap::srv",  // message namespace
  "SpeedControl_Response",  // message name
  1,  // number of fields
  sizeof(pappap::srv::SpeedControl_Response),
  SpeedControl_Response_message_member_array,  // message members
  SpeedControl_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeedControl_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SpeedControl_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeedControl_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pappap


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<pappap::srv::SpeedControl_Response>()
{
  return &::pappap::srv::rosidl_typesupport_introspection_cpp::SpeedControl_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pappap, srv, SpeedControl_Response)() {
  return &::pappap::srv::rosidl_typesupport_introspection_cpp::SpeedControl_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "pappap/srv/detail/speed_control__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace pappap
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers SpeedControl_service_members = {
  "pappap::srv",  // service namespace
  "SpeedControl",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<pappap::srv::SpeedControl>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t SpeedControl_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeedControl_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace pappap


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<pappap::srv::SpeedControl>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::pappap::srv::rosidl_typesupport_introspection_cpp::SpeedControl_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::pappap::srv::SpeedControl_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::pappap::srv::SpeedControl_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, pappap, srv, SpeedControl)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<pappap::srv::SpeedControl>();
}

#ifdef __cplusplus
}
#endif
