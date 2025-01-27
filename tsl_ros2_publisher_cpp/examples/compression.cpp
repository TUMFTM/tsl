// Copyright 2024 Simon Sagmeister

#include <rclcpp/node.hpp>

#include "tsl_logger_cpp/base.hpp"
#include "tsl_logger_cpp/value_logger.hpp"
#include "tsl_ros2_publisher_cpp/tsl_publisher.hpp"
/*
For basic usage of the logger package please first have a look at the examples of tsl_logger_cpp.

This example only focuses on the specifics of the ros2 publisher.
*/

class MySoftwareModule
{
private:
  tam::tsl::ValueLogger::SharedPtr logger = std::make_shared<tam::tsl::ValueLogger>();

public:
  void step()
  {
    logger->log("a", static_cast<bool>(1));
    logger->log("b", static_cast<std::int8_t>(1));
    logger->log("c", static_cast<std::int16_t>(1));
    logger->log("d", static_cast<std::int32_t>(1));
    logger->log("e", static_cast<std::int64_t>(1));
    logger->log("f", static_cast<float>(1));
    logger->log("g", static_cast<double>(1));
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return logger; }
};
class ExampleNode : public rclcpp::Node

{
private:
  MySoftwareModule my_module_{};
  tam::tsl::TSLPublisher::UniquePtr tsl_publisher_{};
  rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&ExampleNode::timer_callback, this));

private:
  void timer_callback()
  {
    my_module_.step();
    tsl_publisher_->trigger();
  }

public:
  ExampleNode() : Node("ExampleNode")
  {
    // In order to save disk space and network bandwith, the TSLPublisher can compress the logged
    // data. This can be controller by constructing the TSLPublisher with a compression strategy
    // object. As a default, the TSLPublisher is constructed with a Compress64to32 bit compression
    // strategy.
    tsl_publisher_ = std::make_unique<tam::tsl::TSLPublisher>(this, my_module_.get_logger());

    // However, if you want to keep this compression strategy, but disable compression for certain
    // signals, you can supply your own compression strategy object
    auto compression_strategy =
      std::make_shared<tam::tsl::builtin_compression_strategies::Compress64to32Bit>();
    compression_strategy->disable_compression("e");  // disable compression for signal "e"
    tsl_publisher_ = std::make_unique<tam::tsl::TSLPublisher>(
      this, my_module_.get_logger(), "", compression_strategy);

    // If you want to disable compression alltogether, you can construct the TSLPublisher with the
    // Uncompressed strategy
    tsl_publisher_ = std::make_unique<tam::tsl::TSLPublisher>(
      this, my_module_.get_logger(), "",
      std::make_shared<tam::tsl::builtin_compression_strategies::Uncompressed>());
  }
};
int main(int argc, char * argv[])
{
  // Create and spin your node.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
}
