# Copyright 2024 Simon Sagmeister
from tsl_logger_py import ValueLogger, register_custom_type


# Define your custom type
class MyCustomType:
    def __init__(self):
        self.a = 1.0
        self.b = 3


# Define anopther custom type
class MyBigCustomType:

    def __init__(self) -> None:
        self.other = MyCustomType()
        self.c = 4.0


# region Add support for your custom types MyCustomType
# ==========================================================
def __log_MyCustomType(logger: ValueLogger, signal: str, value: MyCustomType):
    logger.log(signal + "/a", value.a)
    logger.log(signal + "/b", value.b)


register_custom_type(
    lambda x: isinstance(
        x, MyCustomType
    ),  # Check function if the type support can be applied to a given object
    __log_MyCustomType,  # The type support function that logs the custom type
)


# ==========================================================
# endregion
# Add support for your custom types MyBigCustomType
# ==========================================================
def __log_MyBigCustomType(logger: ValueLogger, signal: str, value: MyBigCustomType):
    logger.log(
        signal + "/other", value.other
    )  # Implictly uses the type support function defined above.
    logger.log(signal + "/c", value.c)


register_custom_type(
    lambda x: isinstance(
        x, MyBigCustomType
    ),  # Check function if the type support can be applied to a given object
    __log_MyBigCustomType,  # The type support function that logs the custom type
)
# ==========================================================
# endregion


# Now your logger is able to log your custom types as well as lists or dicts containing them.

# Create an instance of the logger
logger = ValueLogger()

# Log a custom type
custom_type = MyCustomType()
logger.log("custom_type", custom_type)

# Log a list of custom types
custom_type_list = [MyBigCustomType(), MyBigCustomType()]
logger.log("custom_type_list", custom_type_list)

# Print the logged signals
print("We logged the following signals:")
for signal in logger.get_data():
    print("- " + signal)
