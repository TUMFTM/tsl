# Copyright 2024 Simon Sagmeister
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from tsl_msgs.msg import TSLDefinition, TSLValues
from tsl_logger_py import LoggerAccessInterface
from tsl_ros2_publisher_py.builtin_compression_strategies import Compress64to32Bit
from tsl_ros2_publisher_py._cpp_binding import (
    MsgCreator,
    CompressionStrategy,
    clean_signal_name,
)
from tsl_ros2_publisher_py.definition_trigger import DefinitionTriggerTimer


class TSLPublisher:

    _qos_tsl_definition = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    _qos_tsl_values = _qos_tsl_definition

    def __init__(
        self,
        node_handle: Node,
        logger: LoggerAccessInterface,
        channel_suffix: str = "",
        compression_strategy: CompressionStrategy = Compress64to32Bit(),
    ):
        self._node_handle = node_handle
        self._logger = logger
        self._compression_strategy = compression_strategy

        self._def_trigger = DefinitionTriggerTimer(node_handle)
        self._msg_converter = MsgCreator(self._compression_strategy)

        channel_suffix_cleaned = clean_signal_name(channel_suffix)

        self._definition_publisher = node_handle.create_publisher(
            TSLDefinition,
            f"/tsl{node_handle.get_fully_qualified_name()}{channel_suffix_cleaned}/def",
            self.__class__._qos_tsl_definition,
        )
        self._value_publisher = node_handle.create_publisher(
            TSLValues,
            f"/tsl{node_handle.get_fully_qualified_name()}{channel_suffix_cleaned}",
            self.__class__._qos_tsl_values,
        )

    def trigger(self):
        self._msg_converter.process_data(self._logger)

        if self._def_trigger.due():
            self._definition_publisher.publish(self._msg_converter.get_definition())
            self._def_trigger.register_publishment()  # Register a published def

        self._value_publisher.publish(self._msg_converter.get_values())
