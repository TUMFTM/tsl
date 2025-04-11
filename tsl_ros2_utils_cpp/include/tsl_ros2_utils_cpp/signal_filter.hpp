// Copyright 2025 Simon Sagmeister
#pragma once

#include <algorithm>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <string>
#include <tsl_logger_cpp/base.hpp>
#include <tsl_logger_cpp/types.hpp>
#include <tsl_msgs/msg/tsl_definition.hpp>
#include <tsl_msgs/msg/tsl_values.hpp>
#include <tsl_ros2_publisher_cpp/helpers.hpp>
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

public:
  using UniquePtr = std::unique_ptr<TSLSignalFilter>;

private:
  // QOS Settings
  const rclcpp::QoS qos_tsl_definition_ = rclcpp::QoS(1);
  const rclcpp::QoS qos_tsl_values_ = rclcpp::QoS(1);

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
  void definition_callback(tsl_msgs::msg::TSLDefinition::SharedPtr msg)
  {
    // Create the defininition trigger timer
    definition_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(
        this->get_parameter("timer_periods_ms.definition_trigger").as_int()),
      std::bind(&TSLSignalFilter::definition_trigger, this));

    // Create the value trigger timer
    value_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(this->get_parameter("timer_periods_ms.values_trigger").as_int()),
      std::bind(&TSLSignalFilter::value_trigger, this));

    setup_filter_scheme(*msg);

    // Destroy the subscription to reduce compute requirements in the future
    // Assumption: The definition will not change once subscribed.
    // In the current version - this is ensured by the publisher
    definition_subscriber_ = nullptr;
  }
  void values_callback(std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    values_msg_serialized_ = msg;
  }
  void value_trigger() const
  {
    if (!values_msg_serialized_) {
      return;  // No message recieved yet.
    }

    // Allocate required messages
    tsl_msgs::msg::TSLValues values_msg, values_msg_filtered;
    // Deserialize the values message
    rclcpp::Serialization<tsl_msgs::msg::TSLValues> serializer;
    serializer.deserialize_message(values_msg_serialized_.get(), &values_msg);

    // No need to check if the definition is valid, since the filter scheme is already set up
    // when this timer was created

// Push all the values to the filtered message
#define TSL_FILTER_DTYPE(dtype, typename)                                                  \
  values_msg_filtered.typename##_values.reserve(filter_scheme_.indices_##typename.size()); \
  for (auto const & index : filter_scheme_.indices_##typename) {                           \
    values_msg_filtered.typename##_values.push_back(values_msg.typename##_values[index]);  \
  }
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TSL_FILTER_DTYPE);
#undef TSL_FILTER_DTYPE

    // Set the stamp as well as the hash correctly.
    values_msg_filtered.definition_hash = definition_msg_filtered_->definition_hash;
    values_msg_filtered.stamp = values_msg.stamp;

    // Publish the filtered message
    value_publisher_->publish(values_msg_filtered);
  }
  void definition_trigger() const { definition_publisher_->publish(*definition_msg_filtered_); }
  void setup_filter_scheme(tsl_msgs::msg::TSLDefinition const & msg)
  {
    definition_msg_filtered_ = std::make_shared<tsl_msgs::msg::TSLDefinition>();
    // Set the stamp
    definition_msg_filtered_->stamp = msg.stamp;

    // Get the remaining signals
    auto remaining_signals =
      this->get_parameter("signals_remaining_after_filter").as_string_array();
#define TSL_FIND_INDICES_OF_REMAINING_SIGNALS(dtype, typename)                                  \
  for (std::size_t i = 0; i < msg.typename##_names.size(); ++i) {                               \
    if (                                                                                        \
      std::find(remaining_signals.begin(), remaining_signals.end(), msg.typename##_names[i]) != \
      remaining_signals.end()) {                                                                \
      filter_scheme_.indices_##typename.push_back(i);                                           \
      definition_msg_filtered_->typename##_names.push_back(msg.typename##_names[i]);            \
    }                                                                                           \
  }
    TAM_TSL_FOR_EVERY_SUPPORTED_PRIMITIVE_DATATYPE(TSL_FIND_INDICES_OF_REMAINING_SIGNALS);
#undef TSL_FIND_INDICES_OF_REMAINING_SIGNALS

    definition_msg_filtered_->definition_hash = std::hash<tsl_msgs::msg::TSLDefinition>{}(msg);
  }

public:
  explicit TSLSignalFilter(rclcpp::NodeOptions const & options) : Node("TSL_Signal_Filter", options)
  {
    declare_parameter("timer_periods_ms.definition_trigger", 4000);
    declare_parameter("timer_periods_ms.values_trigger", 100);
    declare_parameter("signals_remaining_after_filter", std::vector<std::string>{});
  }
};
}  // namespace tam::tsl
