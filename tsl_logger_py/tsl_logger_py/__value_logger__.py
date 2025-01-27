# Copyright 2024 Simon Sagmeister
from tsl_logger_py._cpp_binding import ValueLogger as ValueLoggerCpp
from tsl_logger_py.__type_support__ import (
    log_custom_type,
    register_custom_type,
)


class ValueLoggerPy(ValueLoggerCpp):

    def log(self, signal_name, value):
        log_custom_type(self, signal_name, value)
