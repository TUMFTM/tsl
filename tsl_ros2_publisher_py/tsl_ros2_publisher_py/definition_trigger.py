# Copyright 2024 Simon Sagmeister
import rclpy
from rclpy.duration import Duration
import random


class DefinitionTriggerTimer:
    PUBLISH_INTERVAL_SECONDS = Duration(seconds=4, nanoseconds=0)

    def __init__(self, node_handle: rclpy.node.Node):
        self.clock = node_handle.get_clock()
        # Initialize so that the timer is instantly due
        self.last_published = self.clock.now() - Duration(
            nanoseconds=2 * self.PUBLISH_INTERVAL_SECONDS.nanoseconds
        )
        self.initialized = False

    def due(self) -> bool:
        return self.clock.now() - self.last_published >= self.PUBLISH_INTERVAL_SECONDS

    def register_publishment(self):
        if self.initialized:
            self.last_published = self.clock.now()
        else:
            # Offset the timer by a random amount to avoid all definitions being published at the same time
            self.last_published = self.clock.now() - Duration(
                nanoseconds=random.random() * self.PUBLISH_INTERVAL_SECONDS.nanoseconds
            )
            self.initialized = True
