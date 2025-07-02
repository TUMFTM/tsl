// Copyright 2025 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>
#include <tsl_ros2_utils_cpp/ros2_parser.hpp>
namespace tam::tsl
{
class TSLSubscriber
{
  // QOS Settings
  const rclcpp::QoS qos_tsl_definition_ = rclcpp::QoS(1).best_effort();
  const rclcpp::QoS qos_tsl_values_ = rclcpp::QoS(1).best_effort();

public:
  TSLSubscriber(
    rclcpp::Node * node, const std::string & definition_topic, const std::string & values_topic)
  : definition_sub_(node->create_subscription<tsl_msgs::msg::TSLDefinition>(
      definition_topic, qos_tsl_definition_,
      std::bind(&TSLSubscriber::definition_callback, this, std::placeholders::_1))),
    values_sub_(node->create_subscription<tsl_msgs::msg::TSLValues>(
      values_topic, qos_tsl_values_,
      std::bind(&TSLSubscriber::values_callback, this, std::placeholders::_1)))
  {
  }
  ROS2Parser::ConstSharedPtr get_parser();

private:
  void definition_callback(const tsl_msgs::msg::TSLDefinition::SharedPtr msg);
  void values_callback(const tsl_msgs::msg::TSLValues::SharedPtr msg);

private:
  rclcpp::Subscription<tsl_msgs::msg::TSLDefinition>::SharedPtr definition_sub_{nullptr};
  rclcpp::Subscription<tsl_msgs::msg::TSLValues>::SharedPtr values_sub_{nullptr};
  ROS2Parser::SharedPtr parser_{std::make_shared<ROS2Parser>()};
};
}  // namespace tam::tsl
