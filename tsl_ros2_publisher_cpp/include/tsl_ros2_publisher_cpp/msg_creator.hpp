// Copyright 2024 Simon Sagmeister
#pragma once

// clang-format off
#include <rclcpp/rclcpp.hpp>
// #include <tsl_logger_cpp/base.hpp>
// #include "tsl_ros2_publisher_cpp/helpers.hpp"
// clang-format on

#include <rclcpp/serialization.hpp>
#include <string>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>
#include <tsl_ros2_publisher_cpp/compression.hpp>
#include <tsl_ros2_publisher_cpp/helpers.hpp>
#include <unordered_map>
#include <unordered_set>
namespace tam::tsl
{
class MsgCreator
{
private:
  // The number of signals that were present during setting up the definition.
  // We use this as indication that no signals were added later on.
  std::size_t init_size{0};
  // Remember the number of non-logged signals to only print a warning once
  std::size_t last_num_non_logged_signals_{0};
  // Is needed regularly this is why we create the instance once
  rclcpp::Serialization<tsl_msgs::msg::TSLValues> values_serializer_{};
  rclcpp::SerializedMessage def_msg_serialized_{};
  rclcpp::SerializedMessage values_msg_serialized_{};
  tsl_msgs::msg::TSLDefinition def_msg_{};
  tsl_msgs::msg::TSLValues values_msg_{};
  CompressionStrategy::SharedPtr compression_strategy_ = nullptr;

public:
  explicit MsgCreator(CompressionStrategy::SharedPtr compression_strategy)
  : compression_strategy_{compression_strategy} {};
  rclcpp::SerializedMessage const & get_definition() const;
  rclcpp::SerializedMessage const & get_values() const;
  void process_data(data_map_t const & data);

private:
  void print_warning_added_signals(data_map_t const & data);
  void setup_definition_msg(data_map_t const & data);
  void setup_values_msg();
  void update_values_msg(data_map_t const & data);
};  // namespace tam::tsl
}  // namespace tam::tsl
