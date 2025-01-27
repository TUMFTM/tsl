// Copyright 2024 Simon Sagmeister
#pragma once
#ifndef TAM_TSL_BUILTIN_TYPE_SUPPORT_INCLUDED
// Macro to check if the type support was already included.
// This can be used to check for a correct include order of the headers.
#define TAM_TSL_BUILTIN_TYPE_SUPPORT_INCLUDED
#endif

#include <array>
#include <deque>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/type_support_interface.hpp"
#include "tsl_logger_cpp/types.hpp"

#ifndef TAM_TSL_BUILTIN_TYPE_SUPPORT_INCLUDED
// Macro to check if type support is already included
#define TAM_TSL_BUILTIN_TYPE_SUPPORT_INCLUDED
#endif

namespace tam::tsl
{
namespace type_support
{
// ===============================================================================
// region Type support for container types
// ===============================================================================
// Vector
template <typename T>
inline void log(TypeSupportInterface * logger, std::string const & name, std::vector<T> const & vec)
{
  for (std::size_t i = 0; i < vec.size(); i++) {
    log<T>(logger, name + "/" + std::to_string(i), vec[i]);
  }
}
// Array
template <typename T, std::size_t N>
inline void log(
  TypeSupportInterface * logger, std::string const & name, std::array<T, N> const & array)
{
  for (std::size_t i = 0; i < N; i++) {
    log<T>(logger, name + "/" + std::to_string(i), array[i]);
  }
}
// Deque
template <typename T>
inline void log(
  TypeSupportInterface * logger, std::string const & name, std::deque<T> const & deque)
{
  for (std::size_t i = 0; i < deque.size(); i++) {
    log<T>(logger, name + "/" + std::to_string(i), deque[i]);
  }
}
// Map
template <typename T>
inline void log(
  TypeSupportInterface * logger, std::string const & name, std::map<std::string, T> const & map)
{
  for (auto const & [key, value] : map) {
    log<T>(logger, name + "/" + key, value);
  }
}
// Unordered Map
template <typename T>
inline void log(
  TypeSupportInterface * logger, std::string const & name,
  std::unordered_map<std::string, T> const & map)
{
  for (auto const & [key, value] : map) {
    log<T>(logger, name + "/" + key, value);
  }
}
// endregion
// ===============================================================================
// region Type support for non container types
// ===============================================================================
template <typename T>
inline void log(TypeSupportInterface * logger, std::string const &, T const &)
{
  static_assert(
    !std::is_same_v<T, T>,
    "You tried logging a type that is not supported by the type support system of tsl.");
}
// Support type overloads for the builtin types
#define TYPE_SUPPORT_BUILTIN_TYPES(dtype, typename)                               \
  template <>                                                                     \
  inline void log<dtype>(                                                         \
    TypeSupportInterface * logger, std::string const & name, dtype const & value) \
  {                                                                               \
    logger->log(name, value);                                                     \
  }

TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TYPE_SUPPORT_BUILTIN_TYPES)
#undef TYPE_SUPPORT_BUILTIN_TYPES
// Support the builtin variant types
template <>
inline void log<data_variant_t>(
  TypeSupportInterface * logger, std::string const & name, data_variant_t const & value)
{
  logger->log(name, value);
}
}  // namespace type_support
};  // namespace tam::tsl
