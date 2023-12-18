// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from pappap:srv/SpeedControl.idl
// generated code does not contain a copyright notice

#ifndef PAPPAP__SRV__DETAIL__SPEED_CONTROL__TRAITS_HPP_
#define PAPPAP__SRV__DETAIL__SPEED_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "pappap/srv/detail/speed_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace pappap
{

namespace srv
{

inline void to_flow_style_yaml(
  const SpeedControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: spd
  {
    out << "spd: ";
    rosidl_generator_traits::value_to_yaml(msg.spd, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpeedControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: spd
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spd: ";
    rosidl_generator_traits::value_to_yaml(msg.spd, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpeedControl_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pappap

namespace rosidl_generator_traits
{

[[deprecated("use pappap::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pappap::srv::SpeedControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  pappap::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pappap::srv::to_yaml() instead")]]
inline std::string to_yaml(const pappap::srv::SpeedControl_Request & msg)
{
  return pappap::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pappap::srv::SpeedControl_Request>()
{
  return "pappap::srv::SpeedControl_Request";
}

template<>
inline const char * name<pappap::srv::SpeedControl_Request>()
{
  return "pappap/srv/SpeedControl_Request";
}

template<>
struct has_fixed_size<pappap::srv::SpeedControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<pappap::srv::SpeedControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<pappap::srv::SpeedControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace pappap
{

namespace srv
{

inline void to_flow_style_yaml(
  const SpeedControl_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: feedback
  {
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SpeedControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: feedback
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "feedback: ";
    rosidl_generator_traits::value_to_yaml(msg.feedback, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SpeedControl_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace pappap

namespace rosidl_generator_traits
{

[[deprecated("use pappap::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const pappap::srv::SpeedControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  pappap::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use pappap::srv::to_yaml() instead")]]
inline std::string to_yaml(const pappap::srv::SpeedControl_Response & msg)
{
  return pappap::srv::to_yaml(msg);
}

template<>
inline const char * data_type<pappap::srv::SpeedControl_Response>()
{
  return "pappap::srv::SpeedControl_Response";
}

template<>
inline const char * name<pappap::srv::SpeedControl_Response>()
{
  return "pappap/srv/SpeedControl_Response";
}

template<>
struct has_fixed_size<pappap::srv::SpeedControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<pappap::srv::SpeedControl_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<pappap::srv::SpeedControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<pappap::srv::SpeedControl>()
{
  return "pappap::srv::SpeedControl";
}

template<>
inline const char * name<pappap::srv::SpeedControl>()
{
  return "pappap/srv/SpeedControl";
}

template<>
struct has_fixed_size<pappap::srv::SpeedControl>
  : std::integral_constant<
    bool,
    has_fixed_size<pappap::srv::SpeedControl_Request>::value &&
    has_fixed_size<pappap::srv::SpeedControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<pappap::srv::SpeedControl>
  : std::integral_constant<
    bool,
    has_bounded_size<pappap::srv::SpeedControl_Request>::value &&
    has_bounded_size<pappap::srv::SpeedControl_Response>::value
  >
{
};

template<>
struct is_service<pappap::srv::SpeedControl>
  : std::true_type
{
};

template<>
struct is_service_request<pappap::srv::SpeedControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<pappap::srv::SpeedControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PAPPAP__SRV__DETAIL__SPEED_CONTROL__TRAITS_HPP_
