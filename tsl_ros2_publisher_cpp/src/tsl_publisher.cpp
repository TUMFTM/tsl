// Copyright 2024 Simon Sagmeister
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
namespace tam::tsl
{
TSLPublisher::TSLPublisher(
  rclcpp::Node * node_handle, LoggerAccessInterface::SharedPtr logger,
  const std::string & channel_suffix, CompressionStrategy::SharedPtr compression_strategy)
: node_handle_{node_handle}, logger_{logger}, compression_strategy_{compression_strategy}
{
  std::string channel_suffix_cleaned = helpers::clean_signal_name(channel_suffix);
  // Use a generic publisher since this is able to both handle ipc publish and serialized publish
  // If we would use a normal publisher we would have to disable ipc to not throw an exception
  // if the node is part of another executor.
  definition_publisher_ = node_handle->create_generic_publisher(
    std::string("/tsl") + node_handle->get_fully_qualified_name() + channel_suffix_cleaned +
      "/def",
    "tsl_msgs/msg/TSLDefinition", qos_tsl_definition_);
  value_publisher_ = node_handle->create_generic_publisher(
    std::string("/tsl") + node_handle->get_fully_qualified_name() + channel_suffix_cleaned,
    "tsl_msgs/msg/TSLValues", qos_tsl_values_);
}
void TSLPublisher::trigger()
{
  msg_creator_.process_data(logger_->get_data());

  if (def_trigger_.due()) {
    definition_publisher_->publish(msg_creator_.get_definition());
    def_trigger_.register_publishment();  // Register a published def
  }

  value_publisher_->publish(msg_creator_.get_values());
}
}  // namespace tam::tsl
