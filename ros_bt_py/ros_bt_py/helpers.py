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
from enum import StrEnum
from typing import Any
import jsonpickle
import logging

import rclpy
import rclpy.logging
import functools
from collections import OrderedDict

from ros_bt_py_interfaces.msg import CapabilityInterface, Node, Tree
from typeguard import typechecked
from ros_bt_py.ros_helpers import EnumValue, LoggerLevel


@typechecked
class BTNodeState(StrEnum):
    UNINITIALIZED = Node.UNINITIALIZED
    IDLE = Node.IDLE
    UNASSIGNED = Node.UNASSIGNED
    ASSIGNED = Node.ASSIGNED
    RUNNING = Node.RUNNING
    SUCCEEDED = Node.SUCCEEDED
    FAILED = Node.FAILED
    BROKEN = Node.BROKEN
    PAUSED = Node.PAUSED
    SHUTDOWN = Node.SHUTDOWN


@typechecked
def loglevel_is(level) -> bool:
    """
    Determine the current logging level of the default ROS logger.

    Useful to guard log statements that would incur a performance
    penalty if they ran when the log isn't published.

    For easier use, this translates the `rospy` logger levels into
    `logging` levels.

    :param int level:

    The logger level to compare against. Since lower levels are more
    verbose, any level *below* `level` also returns `True`.

    """
    logging_level = logging.getLogger("rosout").getEffectiveLevel()
    return logging_level <= rospy_log_level_to_logging_log_level(level)


@typechecked
def rospy_log_level_to_logging_log_level(rospy_level: int) -> int:
    if rospy_level == rclpy.logging.LoggingSeverity.DEBUG:
        return logging.DEBUG
    if rospy_level == rclpy.logging.LoggingSeverity.INFO:
        return logging.INFO
    if rospy_level == rclpy.logging.LoggingSeverity.WARN:
        return logging.WARN
    if rospy_level == rclpy.logging.LoggingSeverity.ERROR:
        return logging.ERROR
    if rospy_level == rclpy.logging.LoggingSeverity.FATAL:
        return logging.FATAL
    return logging.FATAL


@typechecked
def remove_input_output_values(tree: Tree) -> Tree:
    """
    Remove all input and output values from the tree nodes.

    This is achieved by replacing every nodes input/output serialized_value with "null"
    """
    for node in tree.nodes:
        for node_input in node.inputs:
            node_input.serialized_value = "null"
        for node_output in node.outputs:
            node_output.serialized_value = "null"
    return tree


@typechecked
def set_node_state_to_shutdown(tree: Tree) -> Tree:
    """Set all node states to shutdown."""
    for node in tree.nodes:
        node.state = Node.SHUTDOWN
    return tree


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)

    return functools.reduce(_getattr, [obj] + attr.split("."))


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition(".")
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def get_default_value(data_type: Any, ros=False):
    if data_type is type:
        return int
    elif data_type is int:
        return 0
    elif data_type is str:
        return "foo"
    elif data_type is float:
        return 1.2
    elif data_type is bool:
        return False
    elif data_type is list:
        return []
    elif data_type is dict:
        return {}
    elif data_type is OrderedDict:
        return OrderedDict()
    elif data_type is LoggerLevel:
        return LoggerLevel()
    elif data_type is EnumValue:
        return EnumValue()
    elif ros:
        return data_type()
    else:
        return {}


def json_encode(data):
    """Wrap the call to jsonpickle.encode."""
    return jsonpickle.encode(data)


def json_decode(data):
    """Wrap the call to jsonpickle.decode."""
    return jsonpickle.decode(data)


class MathUnaryOperator(object):
    def __init__(self, operator):
        self.operator = operator


class MathBinaryOperator(object):
    def __init__(self, operator):
        self.operator = operator


class MathOperandType(object):
    def __init__(self, operand_type):
        self.operand_type = operand_type


class MathUnaryOperandType(object):
    def __init__(self, operand_type):
        self.operand_type = operand_type


class HashableCapabilityInterface:
    """Wrapper class to allow for the hashing of capability interfaces."""

    def __init__(self, interface: CapabilityInterface):
        self.interface: CapabilityInterface = interface

    def __eq__(self, other: object) -> bool:
        def compare_node_data_lists(list1: list, list2: list) -> bool:
            l1_node_data = {(x.key, json_decode(x.serialized_type)) for x in list1}
            l2_node_data = {(x.key, json_decode(x.serialized_type)) for x in list2}

            return l1_node_data == l2_node_data

        if not isinstance(other, HashableCapabilityInterface):
            return False

        return (
            (self.interface.name == other.interface.name)
            and compare_node_data_lists(self.interface.inputs, other.interface.inputs)
            and compare_node_data_lists(self.interface.outputs, other.interface.outputs)
            and compare_node_data_lists(self.interface.options, other.interface.options)
        )

    def __ne__(self, other: object) -> bool:
        return not self.__eq__(other)

    def __hash__(self) -> int:
        return hash(
            (
                self.interface.name,
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.inputs
                    }
                ),
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.outputs
                    }
                ),
                frozenset(
                    {
                        (x.key, json_decode(x.serialized_type))
                        for x in self.interface.options
                    }
                ),
            )
        )
