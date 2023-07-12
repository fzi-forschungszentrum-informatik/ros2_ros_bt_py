# Copyright 2018-2023 FZI Forschungszentrum Informatik


import inspect

import rclpy.logging

from ros_bt_py.exceptions import BehaviorTreeException


class LoggerLevel(object):
    """Data class containing a logging level."""

    def __init__(self, logger_level=rclpy.logging.LoggingSeverity.INFO):
        """Initialize a new LoggerLevel class."""
        self.logger_level = logger_level


class EnumValue(object):
    """Data class containing an enum value."""

    def __init__(self, enum_value=""):
        """Initialize a new EnumValue class."""
        self.enum_value = enum_value


def get_message_constant_fields(message_class):
    """Return all constant fields of a message as a list."""
    if inspect.isclass(message_class):
        # This is highly dependend on the ROS message class generation.
        members = [
            attr
            for attr in dir(message_class.__class__)
            if not attr.startswith("_")
            and not callable(getattr(message_class.__class__, attr))
        ]

        return members
    else:
        raise BehaviorTreeException(f"{message_class} is not a ROS Message")
