// Copyright 2025 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>
#include <unordered_map>
#include <vector>
namespace tam::tsl
{
// Development Target:
// Filter a tsl signal to crop out certain signals and republish only the filtered ones
// Use the lowest amount of compute possible.

// A filter to publish filter out certain signals from tsl messages in order to reduce the topic
// bandwidth.

class ROS2Parser
{
public:
#define TAM_TSL_PRINT_ENUM_MEMBER(dtype, typename) ENUM_##typename
  enum DTYPE_ENUM {
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE_WITH_SEPARATOR(
      TAM_TSL_PRINT_ENUM_MEMBER, TAM_TSL_COMMA)
  };
#undef TAM_TSL_PRINT_TYPENAME
  struct SignalLocation
  {
    DTYPE_ENUM signal_type;
    std::size_t index;
  };

using SharedPtr = std::shared_ptr<ROS2Parser>;
using ConstSharedPtr = std::shared_ptr<ROS2Parser const>;

public:
  void set_definition_msg(tsl_msgs::msg::TSLDefinition const & msg);
  void set_definition_msg(tsl_msgs::msg::TSLDefinition::SharedPtr msg);
  void set_values_msg(tsl_msgs::msg::TSLValues const & msg);
  void set_values_msg(tsl_msgs::msg::TSLValues::SharedPtr msg);
  template <typename T>
  T get_value(std::string const & signal_name) const ;
  template <typename T>
  void get_value(std::string const & signal_name, T & value) const;
  bool is_populated() const;

private:
  template <typename T>
  T get_signal_from_cache(SignalLocation const & loc) const;

private:
  // Store messages here for cashing
  tsl_msgs::msg::TSLDefinition::SharedPtr definition_msg_{nullptr};
  // Store message in serialized form to avoid unnecessary deserialization
  tsl_msgs::msg::TSLValues::SharedPtr values_msg_{nullptr};
  mutable std::unordered_map<std::string, SignalLocation> signal_location_cache_;
};
}  // namespace tam::tsl
#include "tsl_ros2_utils_cpp/ros2_parser_impl.hpp"
