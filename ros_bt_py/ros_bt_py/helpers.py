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

#from enum import StrEnum Not available in Python3.10
import abc
from typing import Any, Iterable, Optional
import rclpy
import rclpy.logging
import jsonpickle
import functools
import re
from collections import OrderedDict

import rosidl_runtime_py.utilities

from ros_bt_py_interfaces.msg import NodeState, CapabilityInterface
from typeguard import typechecked
from ros_bt_py.ros_helpers import EnumValue, LoggerLevel


#@typechecked
#class BTNodeState(StrEnum):
class BTNodeState(abc.ABC):
    UNINITIALIZED = NodeState.UNINITIALIZED
    IDLE = NodeState.IDLE
    UNASSIGNED = NodeState.UNASSIGNED
    ASSIGNED = NodeState.ASSIGNED
    RUNNING = NodeState.RUNNING
    SUCCEEDED = NodeState.SUCCEEDED
    FAILED = NodeState.FAILED
    BROKEN = NodeState.BROKEN
    PAUSED = NodeState.PAUSED
    SHUTDOWN = NodeState.SHUTDOWN
BTNodeState.register(str) # Duck tape to satisfy typing. This becomes unnecessary when StrEnum is available


class MathUnaryOperator(object):
    def __init__(self, operator="sqrt"):
        self.operator = operator


class MathBinaryOperator(object):
    def __init__(self, operator="+"):
        self.operator = operator


class MathOperandType(object):
    def __init__(self, operand_type="float"):
        self.operand_type = operand_type


class MathUnaryOperandType(object):
    def __init__(self, operand_type="float"):
        self.operand_type = operand_type


# handling nested objects,
# see https://stackoverflow.com/questions/31174295/getattr-and-setattr-on-nested-objects
def rgetattr(obj, attr, *args):
    def _getattr(obj, attr):
        return getattr(obj, attr, *args)

    return functools.reduce(_getattr, [obj] + attr.split("."))


def rsetattr(obj, attr, val):
    pre, _, post = attr.rpartition(".")
    return setattr(rgetattr(obj, pre) if pre else obj, post, val)


def get_default_value(data_type: Any, ros: bool = False) -> Any:
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


def json_encode(data: Any) -> Optional[str]:
    """Wrap the call to jsonpickle.encode."""
    return jsonpickle.encode(data)


def json_decode(data: str) -> Optional[Any]:
    """Wrap the call to jsonpickle.decode."""
    return jsonpickle.decode(data)


def build_message_field_dicts(message_object: Any) -> tuple[dict, dict]:
    """
    Thin wrapper over `get_field_values_and_types`.

    This allows passing a whole message object,
    this will then iterate over all the message object's fields.

    See the docs of `get_field_values_and_types` for details on what is returned.
    """
    default_message = message_object.__class__()
    values_dict = {}
    types_dict = {}
    for field_name, field_type in message_object.get_fields_and_field_types().items():
        values_dict[field_name], types_dict[field_name] = get_field_values_and_types(
            field_type,
            getattr(message_object, field_name),
            getattr(default_message, field_name),
        )
    return values_dict, types_dict


def get_field_values_and_types(
    field_type: str, field_value: Optional[Any], optional_default: Optional[Any] = None
) -> tuple[Any, dict]:
    """
    Recursively identify the values and types of all fields.

    Given a corresponding type string, current value and (optional) default value.
    The default_value is used to obtain custom defaults set at message definition,
    if omitted, it will be recovered where possible, and fallbacks will be used where it's not.

    The field values are returned as a simple recursive dictionary,
    ending in values of buit-in types.
    These are only valid if field_value is not None

    The field types are given as a dictionary with three keys:
        - 'own_type': the type of the field, may be one of the following:
            - a built-in type
            - a message class name
            - the special keyword 'sequence' indicating an array field
        - 'default_value': the default value of the built-in type, None otherwise
        - 'nested_type': depending on the value of own_type, this is:
            - None for built-in types
            - a (recursive) dict with field names and types for message classes
            - a dict with a type specification (this) for 'sequence'
        - 'max_length': only relevant for 'sequence' and 'string' types,
            equals -1 if it's unbounded and for all other types
        - 'is_static': specifies if the max_length of 'sequence' is forced,
            meaing the sequence has to be exactly this length. False for all other types
    """
    # Checks if the type matches a sequence definition and extracts the element-type and bounds
    match_sequence = re.match(r"sequence<([\w\/<>]+)(?:, (\d+))?>", field_type)
    # Checks if the type matches an array definition and extracts the element-type and bounds
    match_array = re.match(r"([\w\/<>]*)\[(\d+)\]", field_type)
    # Parse sequence and array types
    if match_sequence or match_array:
        # Recover defaults
        default_value = optional_default
        if optional_default is None:
            default_value = []

        nested_type_str: str
        max_len_str: str
        is_static = False
        if match_sequence:
            nested_type_str, max_len_str = match_sequence.groups()
        if match_array:
            nested_type_str, max_len_str = match_array.groups()
            is_static = True
        if max_len_str is None:
            max_len = -1
        else:
            max_len = int(max_len_str)
        _, nested_type = get_field_values_and_types(nested_type_str, None)
        nested_values = []
        if field_value is not None:
            for val in field_value:
                nval, _ = get_field_values_and_types(nested_type_str, val)
                nested_values.append(nval)
        return nested_values, {
            "own_type": field_type,
            "default_value": None,
            "nested_type": nested_type,
            "max_length": max_len,
            "is_static": is_static,
        }

    # Check if the type matches a message type
    if field_type.find("/") != -1:
        # Get default field values
        default_value = optional_default
        if optional_default is None:
            default_value = rosidl_runtime_py.utilities.get_message(field_type)()

        default_value: Any

        # Iterate fields for recursion
        nested_value = {}
        nested_type = {}
        for fname, ftype in default_value.get_fields_and_field_types().items():
            nested_value[fname], nested_type[fname] = get_field_values_and_types(
                ftype,
                getattr(field_value, fname, None),
                getattr(default_value, fname),
            )
        return nested_value, {
            "own_type": field_type,
            "default_value": None,
            "nested_type": nested_type,
            "max_length": -1,
            "is_static": False,
        }

    # Parse built-in types
    fallback_default: Any = None
    max_len = -1
    if field_type.find("bool") != -1:
        fallback_default = False
    elif (
        field_type.find("int") != -1
        or field_type.find("long") != -1
        or field_type.find("char") != -1
    ):
        fallback_default = 0
    elif field_type.find("float") != -1 or field_type.find("double") != -1:
        fallback_default = 0.0
    elif field_type.find("octet") != -1:
        fallback_default = b"\x00"
    elif field_type.find("string") != -1:
        fallback_default = ""
        match_length = re.match(r"\w+<(\d+)>", field_type)
        if match_length:
            max_len = int(match_length[1])
    else:
        rclpy.logging.get_logger("package_manager").warn(
            f"Unidentified built-in type {field_type}"
        )

    default_value = optional_default
    if optional_default is None:
        default_value = fallback_default

    return field_value, {
        "own_type": field_type,
        "default_value": default_value,
        "nested_type": None,
        "max_length": max_len,
        "is_static": False,
    }


class HashableCapabilityInterface:
    """Wrapper class to allow for the hashing of capability interfaces."""

    def __init__(self, interface: CapabilityInterface):
        self.interface: CapabilityInterface = interface

    def __eq__(self, other: object) -> bool:
        def compare_node_data_lists(list1: Iterable, list2: Iterable) -> bool:
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
