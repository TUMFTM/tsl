// Copyright 2025 Simon Sagmeister
#include "tsl_ros2_utils_cpp/ros2_parser.hpp"
namespace tam::tsl
{
void ROS2Parser::set_definition_msg(tsl_msgs::msg::TSLDefinition const & msg)
{
  set_definition_msg(std::make_shared<tsl_msgs::msg::TSLDefinition>(msg));
}
void ROS2Parser::set_definition_msg(tsl_msgs::msg::TSLDefinition::SharedPtr msg)
{
  // Check if the same definitnion was reset
  if (definition_msg_) {
    // If the definition message is not null, check if the hash is the same
    if (definition_msg_->definition_hash == msg->definition_hash) {
      return;  // Definition message is the same, no need to update
    }
  }

  // Invalidate the cache first
  signal_location_cache_.clear();
  definition_msg_ = msg;
}
void ROS2Parser::set_values_msg(tsl_msgs::msg::TSLValues const & msg)
{
  set_values_msg(std::make_shared<tsl_msgs::msg::TSLValues>(msg));
}
void ROS2Parser::set_values_msg(tsl_msgs::msg::TSLValues::SharedPtr msg) { values_msg_ = msg; }
bool ROS2Parser::is_populated() const { return (definition_msg_ && values_msg_); }
}  // namespace tam::tsl
