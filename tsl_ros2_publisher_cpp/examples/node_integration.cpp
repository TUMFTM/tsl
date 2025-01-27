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
  int a = 2, b = 3;
  tam::tsl::ValueLogger::SharedPtr logger = std::make_shared<tam::tsl::ValueLogger>();

public:
  void step()
  {
    int result = a * b;
    logger->log("result", result);
    logger->log("a", a);
    logger->log("b", b);
  }
  tam::tsl::LoggerAccessInterface::SharedPtr get_logger() { return logger; }
};
class ExampleNode : public rclcpp::Node

{
private:
  MySoftwareModule my_module_{};
  // Create a tsl publisher by passing the node and the logger
  // This persistently connects your logger to the publisher
  tam::tsl::TSLPublisher tsl_publisher_{this, my_module_.get_logger()};
  rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&ExampleNode::timer_callback, this));

private:
  void timer_callback()
  {
    my_module_.step();
    // Call the tsl_publisher's trigger function to publish your logged values.
    // This also publishes the definition whenever it is needed.
    // The first call to trigger also locks the definition for which signals will be logged and
    // their respective datatypes. So be sure you've logged all signals before calling trigger.
    tsl_publisher_.trigger();
  }

public:
  ExampleNode() : Node("ExampleNode") {}
};
int main(int argc, char * argv[])
{
  // Create and spin your node.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
}
