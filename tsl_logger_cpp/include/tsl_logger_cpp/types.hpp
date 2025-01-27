// Copyright 2024 Simon Sagmeister
#pragma once

#include <string>
#include <unordered_map>
#include <variant>
namespace tam::tsl
{

// region define macros for the pkg
#define TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE_WITH_SEPARATOR(CALLABLE_MACRO, SEPARATOR) \
  CALLABLE_MACRO(bool, bool) SEPARATOR CALLABLE_MACRO(std::int8_t, int8)                         \
  SEPARATOR                                                                                      \
  CALLABLE_MACRO(std::uint8_t, uint8) SEPARATOR CALLABLE_MACRO(std::int16_t, int16)              \
  SEPARATOR                                                                                      \
  CALLABLE_MACRO(std::uint16_t, uint16) SEPARATOR CALLABLE_MACRO(std::int32_t, int32)            \
  SEPARATOR                                                                                      \
  CALLABLE_MACRO(std::uint32_t, uint32) SEPARATOR CALLABLE_MACRO(std::int64_t, int64)            \
  SEPARATOR                                                                                      \
  CALLABLE_MACRO(std::uint64_t, uint64) SEPARATOR CALLABLE_MACRO(float, float32)                 \
  SEPARATOR                                                                                      \
  CALLABLE_MACRO(double, float64)

#define TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(CALLABLE_MACRO) \
  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE_WITH_SEPARATOR(CALLABLE_MACRO, )

#define TAM_TSL_COMMA ,
#define TAM_TSL_PRINT_DTYPE(dtype, typename) dtype
#define TAM_TSL_PRINT_DTYPE_CONST_PTR(dtype, typename) dtype const *
// endregion

using data_variant_t = std::variant<TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE_WITH_SEPARATOR(
  TAM_TSL_PRINT_DTYPE, TAM_TSL_COMMA)>;
using data_map_t = std::unordered_map<std::string, data_variant_t>;

#undef TAM_TSL_PRINT_DTYPE
}  // namespace tam::tsl
