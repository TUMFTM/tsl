// Copyright 2025 Simon Sagmeister
#include "tsl_ros2_utils_cpp/tsl_subscriber.hpp"
namespace tam::tsl
{
    void TSLSubscriber::definition_callback(const tsl_msgs::msg::TSLDefinition::SharedPtr msg)
    {
      parser_->set_definition_msg(msg);
      // Destroy the subscription then
      definition_sub_ = nullptr;
    }
    void TSLSubscriber::values_callback(const tsl_msgs::msg::TSLValues::SharedPtr msg)
    {
      parser_->set_values_msg(msg);
    }
    ROS2Parser::ConstSharedPtr TSLSubscriber::get_parser() { return parser_; }
}