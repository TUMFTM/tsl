// Copyright 2025 Simon Sagmeister
#pragma once
#include <memory>
#include <string>
#include <tsl_ros2_utils_cpp/ros2_parser.hpp>
namespace tam::tsl
{

template <typename T>
T ROS2Parser::get_value(std::string const & signal_name) const
{
  // First check that the hashes of the definition and the values message align
   if (!definition_msg_) {
    throw std::runtime_error("Missing TSL Definition");
  }
  if (!values_msg_) {
    throw std::runtime_error("Missing TSL Values");
  }
  if (definition_msg_->definition_hash != values_msg_->definition_hash) {
    throw std::runtime_error("TSL Definition and Values do not match");
  }

  // First check if a signal location is already cached
  auto it = signal_location_cache_.find(signal_name);
  if (it != signal_location_cache_.end()) {
    // Signal location is cached, use it
    auto const & signal_location = it->second;
    return get_signal_from_cache<T>(signal_location);
  }
  // Signal location is not cached, find it in the definition message
#define TAM_TSL_LOOKUP_MATCHING_SIGNAL(dtype, typename)                        \
  for (std::size_t i = 0; i < definition_msg_->typename##_names.size(); ++i) { \
    if (signal_name == definition_msg_->typename##_names[i]) {                 \
      signal_location_cache_[signal_name] = {DTYPE_ENUM::ENUM_##typename, i};  \
      return values_msg_->typename##_values[i];                                \
    }                                                                          \
  }
#define TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_MATCHING_TYPE(dtype, typename) \
  if constexpr (std::is_same_v<T, dtype>) {                                \
    TAM_TSL_LOOKUP_MATCHING_SIGNAL(dtype, typename)                        \
  }

#define TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_DIFFERENT_TYPE(dtype, typename) \
  if constexpr (!std::is_same_v<T, dtype>) {                                 \
    TAM_TSL_LOOKUP_MATCHING_SIGNAL(dtype, typename)                         \
  }

  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_MATCHING_TYPE)
  TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(
    TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_DIFFERENT_TYPE);

#undef TAM_TSL_LOOKUP_MATCHING_SIGNAL
#undef TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_MATCHING_TYPE
#undef TAM_TSL_LOOKUP_MATCHING_SIGNAL_WITH_DIFFERENT_TYPE
  // If we reach this point, the signal was not found
  throw std::out_of_range(
    std::string("Signal not contained within the tsl definition: ") + signal_name);
}
template <typename T>
T ROS2Parser::get_signal_from_cache(ROS2Parser::SignalLocation const & loc) const
{
#define TAM_TSL_MATCH_ENUM_TYPE(dtype, typename) \
  case DTYPE_ENUM::ENUM_##typename:              \
    return static_cast<T>(values_msg_->typename##_values[loc.index]);
  switch (loc.signal_type) {
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TAM_TSL_MATCH_ENUM_TYPE)
  }
#undef TAM_TSL_MATCH_ENUM_TYPE
  throw std::runtime_error(
    std::string("Signal has a nonexistant datatype cached"));  // Should never happen
}
template <typename T>
void ROS2Parser::get_value(std::string const & signal_name, T & value) const
{
  value = get_value<T>(signal_name);
}
}  // namespace tam::tsl
