# Copyright 2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import inspect
import array
import uuid

import rclpy.logging
from rclpy import action
from rclpy.node import Node, Publisher

from unique_identifier_msgs.msg import UUID as ROS_UUID

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py_interfaces.msg import MessageChannel, MessageChannels


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


def ros_to_uuid(ros_uuid_msg):
    return uuid.UUID(bytes=bytes(ros_uuid_msg.uuid))


def uuid_to_ros(uuid):
    ros_uuid = ROS_UUID()
    ros_uuid.uuid = list(uuid.bytes)  # Convert UUID bytes back to list
    return ros_uuid


def get_interface_name(msg_metaclass: type) -> str:
    """
    Extract the interface name from a ROS2 message metaclass.

    :param msg_metaclass: The ROS2 message metaclass.

    :returns: The interface name in the format 'package_name/message_type/message_name'.
    """
    module_parts = msg_metaclass.__module__.split(".")
    package_name = module_parts[0]
    message_type = module_parts[-2]  # Extract 'msg', 'srv', or 'action'
    message_name = msg_metaclass.__name__
    return f"{package_name}/{message_type}/{message_name}"


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


def publish_message_channels(node: Node, publisher: Publisher):
    """Return all known topic-, service-, and action-names."""
    msg = MessageChannels()
    # These reassignments makes the typing happy,
    #   because they ensure that `.append` exists
    msg.topics = []
    msg.services = []
    msg.actions = []
    # Types are returned as 1?-element lists, so we need to unpack them
    for name, [interface, *_] in node.get_topic_names_and_types():
        msg.topics.append(MessageChannel(name=name, type=interface))
    for name, [interface, *_] in node.get_service_names_and_types():
        msg.services.append(MessageChannel(name=name, type=interface))
    for name, [interface, *_] in action.get_action_names_and_types(node):
        msg.actions.append(MessageChannel(name=name, type=interface))
    publisher.publish(msg)


def get_message_field_type(msg, field) -> type:
    """Return type of a field in a ros msg but check for arrays to be converted into lists."""
    if isinstance((getattr(msg, field)), array.array):
        return list
    else:
        return type(getattr(msg, field))
