// Copyright 2024 Simon Sagmeister
#pragma once
#include <rclcpp/node.hpp>
// This module has to return true if the definition should be published.

namespace tam::tsl
{
class DefintionTriggerTimer
{
private:
  const rclcpp::Duration PUBLISH_INTERVAL{4, 0};
  rclcpp::Clock::SharedPtr clock_;
  // Initialize so that the timer is instantly due
  rclcpp::Time last_published_{clock_->now() - PUBLISH_INTERVAL * 2};
  bool initialized_{false};

public:
  explicit DefintionTriggerTimer(rclcpp::Node * node_handle) : clock_{node_handle->get_clock()} {}
  bool due() { return clock_->now() - last_published_ >= PUBLISH_INTERVAL; }
  void register_publishment()
  {
    if (initialized_) {
      last_published_ = clock_->now();
    } else {
      // Offset the timer by a random amount to avoid all definitions being published at the same
      // time
      last_published_ =
        clock_->now() - PUBLISH_INTERVAL * (static_cast<float>(rand()) / RAND_MAX);  // NOLINT
      initialized_ = true;
    }
  }
};
}  // namespace tam::tsl
