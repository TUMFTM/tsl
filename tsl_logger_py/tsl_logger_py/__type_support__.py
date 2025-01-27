# Copyright 2024 Simon Sagmeister
from typing import Callable, List, Tuple

from tsl_logger_py.exceptions import UnsupportedType

# region type support backend
__tam_tsl_type_support__: List[
    Tuple[Callable[[object], bool], Callable[["ValueLogger", str, object], None]]
] = list()


def register_custom_type(
    type_support_detector: Callable[[object], bool],
    type_support_function: Callable[["ValueLogger", str, object], None],
):
    """Register a type support function for a custom type.

    Args:
        type_support_detector (Callable[[object], bool]): A function that returns True if the type support function should be called for a given object.
        type_support_function (Callable[[&quot;ValueLogger&quot;, str, object], None]): The type support function that logs the custom type.
    """
    __tam_tsl_type_support__.append((type_support_detector, type_support_function))


def log_custom_type(logger: "ValueLogger", signal_name: str, value: object):
    for type_check_function, type_support_function in __tam_tsl_type_support__:
        if type_check_function(value):
            type_support_function(logger, signal_name, value)
            return
    raise Exception(
        f"tsl_logger_py | There is no type support for an object of type: {value.__class__.__name__}"
    )


# Builtin supportet types
register_custom_type(
    lambda x: isinstance(x, float),
    lambda logger, signal, value: logger.log_float64(signal, value),
)
register_custom_type(
    lambda x: isinstance(x, int),
    lambda logger, signal, value: logger.log_int64(signal, value),
)


def log_iterable(logger, signal, iterable_value):
    for index, value in enumerate(iterable_value):
        logger.log(signal + "/" + str(index), value)


register_custom_type(
    lambda x: isinstance(x, list),
    log_iterable,
)


def log_dict(logger, signal, dict_value):
    for key, value in dict_value.items():
        logger.log(signal + "/" + key, value)


register_custom_type(
    lambda x: isinstance(x, dict),
    log_dict,
)
