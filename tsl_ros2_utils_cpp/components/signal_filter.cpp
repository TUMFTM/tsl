// Copyright 2025 Simon Sagmeister
#include "tsl_ros2_utils_cpp/signal_filter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
namespace tsl_ros2_utils_cpp
{
using SignalFilter = tam::tsl::TSLSignalFilter;
};  // namespace tsl_ros2_utils_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(tsl_ros2_utils_cpp::SignalFilter)
