# Copyright 2024 Simon Sagmeister
from tsl_logger_py import ValueLogger


logger1 = ValueLogger()

# You can just log any numeric value
logger1.log("int_test", 2)
logger1.log("float_test", 3.0)

# You can even log lists or dicts
vec = [
    5.0,
    6.0,
    7.0,
]  # Avoid mixing floats and ints in a vectors. Even though it is possible.
map = {"signal_a": 1, "signal_b": 2}
logger1.log("Testvector", vec)
logger1.log("Testmap", map)


# WARNING: Everything that is logged with the logger.log() function will be logged as 64 bit datatype

# This is due to to the dynamic sizing of python numeric types. So its not possible to determine
# which integer/floating type to use.
# However, you can explicitly specify a datatype that will be used for logging your value
logger1.log_float64("float64_test", 6.0)
logger1.log_float32("float32_test", 6.0)
logger1.log_uint64("uint64_test", 5)
logger1.log_int64("int64_test", 5)
logger1.log_uint32("uint32_test", 5)
logger1.log_int32("int32_test", 5)
logger1.log_uint16("uint16_test", 5)
logger1.log_int16("int16_test", 5)
logger1.log_uint8("uint8_test", 5)
logger1.log_int8("int8_test", 5)


# You can even log the values contained in another logger
# Note: This will copy the data over. For a faster way of combining loggers, see the composition
# example
logger2 = ValueLogger()
logger2.log("logger1_data", logger1.get_data())

print("We logged the following signals in Logger 1:")
print(type(logger1.get_data()))
for name, value in logger1.get_data().items():
    print("- " + name + f": {value}")

print("\n\n\n")

print("We logged the following signals in Logger 2:")
for name, value in logger2.get_data().items():
    print("- " + name + f": {value}")
