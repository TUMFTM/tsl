// Copyright 2025 Simon Sagmeister
#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>
#include <vector>
namespace tam::tsl
{
// Development Target:
// Filter a tsl signal to crop out certain signals and republish only the filtered ones
// Use the lowest amount of compute possible.

// A filter to publish filter out certain signals from tsl messages in order to reduce the topic
// bandwidth.
class TSLSignalFilter : public rclcpp::Node
{
  struct FilterScheme
  {
#define TSL_DEFINE_INDEX_FILTER(dtype, typename) std::vector<std::size_t> indices_##typename {};
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TSL_DEFINE_INDEX_FILTER)
#undef TSL_DEFINE_INDEX_FILTER
  };

private:
  // QOS Settings
  const rclcpp::QoS qos_tsl_definition_ = rclcpp::QoS(1).best_effort();
  const rclcpp::QoS qos_tsl_values_ = rclcpp::QoS(1).best_effort();

private:
  // Store messages here for cashing
  tsl_msgs::msg::TSLDefinition::SharedPtr definition_msg_filtered_{nullptr};
  // Store message in serialized form to avoid unnecessary deserialization
  std::shared_ptr<rclcpp::SerializedMessage> values_msg_serialized_{nullptr};
  FilterScheme filter_scheme_{};

private:
  // Create all the node entities: timers, subscribers, publishers

  // Declare timers - Init them to zero.
  // Only publish once a message was recieved.
  rclcpp::TimerBase::SharedPtr definition_timer_{nullptr};
  rclcpp::TimerBase::SharedPtr value_timer_{nullptr};

  // Declare the corresponding subscribers
  rclcpp::Subscription<tsl_msgs::msg::TSLDefinition>::SharedPtr definition_subscriber_{
    create_subscription<tsl_msgs::msg::TSLDefinition>(
      declare_parameter("topics.subscription.definition", "/tsl/node_name/def"),
      qos_tsl_definition_,
      std::bind(&TSLSignalFilter::definition_callback, this, std::placeholders::_1))};
  rclcpp::GenericSubscription::SharedPtr value_subscriber_{create_generic_subscription(
    declare_parameter("topics.subscription.values", "/tsl/node_name"), "tsl_msgs/msg/TSLValues",
    qos_tsl_values_, std::bind(&TSLSignalFilter::values_callback, this, std::placeholders::_1))};

  // Declare the corresponding publishers
  rclcpp::Publisher<tsl_msgs::msg::TSLDefinition>::SharedPtr definition_publisher_{
    create_publisher<tsl_msgs::msg::TSLDefinition>(
      declare_parameter(
        "topics.publisher.definition",
        get_parameter("topics.subscription.definition").as_string() + "/filtered"),
      qos_tsl_definition_)};
  rclcpp::Publisher<tsl_msgs::msg::TSLValues>::SharedPtr value_publisher_{
    create_publisher<tsl_msgs::msg::TSLValues>(
      declare_parameter(
        "topics.publisher.values",
        get_parameter("topics.subscription.values").as_string() + "/filtered"),
      qos_tsl_values_)};

private:
  void definition_callback(tsl_msgs::msg::TSLDefinition::SharedPtr msg);
  void values_callback(std::shared_ptr<rclcpp::SerializedMessage> msg);
  void value_trigger() const;
  void definition_trigger() const;
  void setup_filter_scheme(tsl_msgs::msg::TSLDefinition const & msg);

public:
  explicit TSLSignalFilter(rclcpp::NodeOptions const & options);
};
}  // namespace tam::tsl
