// Copyright 2024 Simon Sagmeister
#pragma once

#include <algorithm>
#include <regex>
#include <string>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
namespace std
{
template <>
struct hash<tsl_msgs::msg::TSLDefinition>
{
  std::size_t operator()(const tsl_msgs::msg::TSLDefinition & def) const noexcept
  {
    std::string hash_string = "";
    std::string type_delimiter = "\n";
    std::string name_delimiter = "|";

    // clang-format off
    #define ACCUMULATE_HASH_STRING(dtype, typename)  \
      for (auto const & name : def.typename##_names) {  \
        hash_string += name + name_delimiter;  \
      }  \
      hash_string += type_delimiter;
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(ACCUMULATE_HASH_STRING)
    #undef ACCUMULATE_HASH_STRING
    // clang-format on
    return std::hash<std::string>{}(hash_string);
  }
};
}  // namespace std
namespace tam::tsl::helpers
{
template <typename T>
struct NumericVariantCastVisitor
{
  // clang-format off
  #define TYPE_CAST_OVERLOAD(dtype, name) T operator()(dtype const & value) const { return value;}
  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TYPE_CAST_OVERLOAD)
  #undef TYPE_CAST_OVERLOAD
  // clang-format on
};
template <typename T>
T numeric_variant_cast(data_variant_t const & variant)
{
  return std::visit(NumericVariantCastVisitor<T>{}, variant);
}
inline std::string clean_signal_name(std::string name)
{
  if (name.empty()) {
    return name;
  }
  std::string name_copy;
  name = std::regex_replace(name, std::regex("\\s+"), "$1");  // Remove whitespaces
  name.erase(0, std::min(name.find_first_not_of("/"),
                         name.size() - 1));                       // Remove leading slashes
  name.erase(name.find_last_not_of("/") + 1, std::string::npos);  // Remove trailing slash

  if (name.empty()) {
    throw std::runtime_error(std::string("The signal name: '") + name_copy + "' is not valid");
  }
  return name;
}
}  // namespace tam::tsl::helpers
