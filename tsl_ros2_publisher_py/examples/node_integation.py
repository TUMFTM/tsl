# Copyright 2024 Simon Sagmeister

import rclpy
from rclpy.node import Node
from tsl_logger_py import LoggerAccessInterface, ValueLogger
from tsl_ros2_publisher_py import TSLPublisher
from tsl_ros2_publisher_py.builtin_compression_strategies import Uncompressed

"""
For basic usage of the logger package please first have a look at the examples of tsl_logger_py.

This example only focuses on the specifics of the ros2 publisher.
"""


class MySoftwareModule:
    def __init__(self):
        self.a = 2
        self.b = 3
        self.logger = ValueLogger()

    def step(self):
        result = self.a * self.b
        self.logger.log("result", result)
        self.logger.log("a", self.a)
        self.logger.log("b", self.b)
        self.logger.log("a_detail", 2.0)
        self.logger.log_uint8("status", 3)

    def get_logger(self) -> LoggerAccessInterface:
        return self.logger


class ExampleNode(Node):
    def __init__(self):
        super().__init__("ExampleNode")
        self.my_module = MySoftwareModule()
        # Create a tsl publisher by passing the node and the logger
        # This persistently connects your signal to the publisher.
        self.tsl_publisher = TSLPublisher(
            self,
            self.my_module.get_logger(),
            # compression_strategy=Uncompressed() # Optional: Set a compression strategy.
            # For details see the compression example.
        )
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        self.my_module.step()
        # Call the tsl_publisher's trigger function to publish your logged values.
        # This also publishes the definition whenever it is needed.
        # The first call to trigger also locks the definition for which signals will be logged and
        # their respective datatypes. So be sure you've logged all signals before calling trigger.
        self.tsl_publisher.trigger()


def main(args=None):
    # Create and spin your node.
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
