// Copyright 2024 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tsl_logger_cpp/base.hpp>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>

#include "tsl_ros2_publisher_cpp/definition_trigger.hpp"
#include "tsl_ros2_publisher_cpp/helpers.hpp"
#include "tsl_ros2_publisher_cpp/msg_creator.hpp"
namespace tam::tsl
{
// A publisher to publish your time series logging data.
class TSLPublisher
{
  // PKG Configs
public:
  using UniquePtr = std::unique_ptr<TSLPublisher>;

private:
  const rclcpp::QoS qos_tsl_definition_ = rclcpp::QoS(1);
  const rclcpp::QoS qos_tsl_values_ = rclcpp::QoS(1);

  // Storage for the ros2 entities
private:
  rclcpp::Node * node_handle_{};
  // Use a generic publisher since this is able to both handle ipc publish and serialized publish
  // If we would use a normal publisher we would have to disable ipc to not throw an exception
  // if the node is part of another executor.
  rclcpp::GenericPublisher::SharedPtr definition_publisher_{};
  rclcpp::GenericPublisher::SharedPtr value_publisher_{};

  // Storage for the tam::tsl entities
private:
  LoggerAccessInterface::SharedPtr logger_{};
  DefintionTriggerTimer def_trigger_{node_handle_};
  CompressionStrategy::SharedPtr compression_strategy_{};
  MsgCreator msg_creator_{compression_strategy_};

public:
  /// @brief Create a ros2 time series logging publisher.
  /// @brief The topics published by this publisher will be:
  /// @brief - `/tsl/${fully_qualified_node_name}/${channel_suffix}/def`
  /// @brief - `/tsl/${fully_qualified_node_name}/${channel_suffix}`
  /// @param node_handle
  /// @param logger
  /// @param channel_suffix
  /// @param compression_strategy
  explicit TSLPublisher(
    rclcpp::Node * node_handle, LoggerAccessInterface::SharedPtr logger,
    const std::string & channel_suffix = "",
    CompressionStrategy::SharedPtr compression_strategy =
      std::make_shared<builtin_compression_strategies::Compress64to32Bit>());
  /// @brief Publish the logged data. This function also publishes the definition whenever it is
  /// necessary.
  /// @brief The first call of this function will lock the definition for which signals will be
  /// logged and their respective datatypes.
  /// @brief So be sure you've logged all signals before calling trigger the first time.
  void trigger();
};
}  // namespace tam::tsl
