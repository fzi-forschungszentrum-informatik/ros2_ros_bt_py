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
from copy import deepcopy
from typing import Any, Optional, TypeVar

from typeguard import typechecked

import itertools

from ros_bt_py.vendor.result import Result, Err, Ok

from ros_bt_py.data_types import DataContainer, TypeContainerMixin, ReferenceContainer
from ros_bt_py.exceptions import NodeConfigError


@typechecked
class NodeDataMap:
    """
    This wrapper around a plain `dict[str, DataContainer]`
    for easier access to value and updated status and easier error handling.
    """

    def __init__(self, name: str, data: dict[str, DataContainer]) -> None:
        self.name = name
        self.data = data

    def _get_item(self, key: str) -> Result[DataContainer, NodeConfigError]:
        if key not in self.data.keys():
            return Err(NodeConfigError(f"Key {key} does not exist in {self.name}"))
        return Ok(self.data[key])


T = TypeVar("T")


@typechecked
class NodeInputMap(NodeDataMap):

    def get_value(self, key: str) -> Result[Any, NodeConfigError]:
        match self._get_item(key):
            case Err(e):
                return Err(e)
            case Ok(c):
                container = c
        match container.get_value():
            case Err(None):
                return Err(
                    NodeConfigError(f"No value set for key {key} in {self.name}")
                )
            case Ok(v):
                return Ok(v)

    def get_value_as(self, key: str, type_: type[T]) -> Result[T, NodeConfigError]:
        match self._get_item(key):
            case Err(e):
                return Err(e)
            case Ok(c):
                container = c
        match container.get_value_as(type_):
            case Err(v):
                return Err(
                    NodeConfigError(
                        f"Value {v} for key {key} in {self.name} is not of type {type_}"
                    )
                )
            case Ok(v):
                return Ok(v)

    def any_updated(self, *keys: str) -> Result[bool, NodeConfigError]:
        updated = False
        for key in keys:
            match self._get_item(key):
                case Err(e):
                    return Err(e)
                case Ok(c):
                    container = c
            if container.is_updated():
                updated = True
        return Ok(updated)


@typechecked
class NodeOutputMap(NodeDataMap):

    def set_value(self, key: str, value: Any) -> Result[None, NodeConfigError]:
        match self._get_item(key):
            case Err(e):
                return Err(e)
            case Ok(c):
                container = c
        match container.set_value(value):
            case Err(s):
                return Err(
                    NodeConfigError(
                        f"Error setting value {value} for key {key} in {self.name}: {s}"
                    )
                )
            case Ok(None):
                return Ok(None)

    def set_multiple_values(self, **value_dict: Any) -> Result[None, NodeConfigError]:
        for key, value in value_dict.items():
            match self.set_value(key, value):
                case Err(e):
                    return Err(e)
                case Ok(None):
                    pass
        return Ok(None)


@typechecked
class NodeConfig:

    def __init__(
        self,
        inputs: dict[str, DataContainer],
        outputs: dict[str, DataContainer],
        max_children: Optional[int],
        tags: Optional[list[str]] = None,
    ):
        """
        Describe the interface of a :class:ros_bt_py.node.Node .

        :type inputs Dict[str, DataContainer]
        :param inputs:

        Map from input names to their data types.

        :type outputs Dict[str, DataContainer]
        :param outputs:

        Map from output names to their data types.

        :type max_children: int or None
        :param max_children:

        The maximum number of children this node type is allowed to
        have.  If this is None, the node type supports any amount of
        children.
        """
        self.inputs = inputs
        self.outputs = outputs
        self.max_children = max_children

        if tags is None:
            tags = []
        self.tags = tags

        # For convenience, all outputs are silently forced to be dynamic only
        # If an output is an instance of `TypeContainerMixin`, raise `NodeConfigError`
        for output in self.outputs.values():
            if isinstance(output, TypeContainerMixin):
                raise NodeConfigError("Cannot have a TypeContainer as an output")
            output.allow_dynamic = True
            output.allow_static = False
            output.is_static = False

        # Initialize all instances of `ReferenceContainer`
        for io_item in itertools.chain(self.inputs.values(), self.outputs.values()):
            if not isinstance(io_item, ReferenceContainer):
                continue
            match io_item.set_type_map(self.inputs):
                case Err(e):
                    raise NodeConfigError(e)
                case Ok(None):
                    pass

    def __repr__(self) -> str:
        return (
            "NodeConfig("
            f"inputs={self.inputs}, "
            f"outputs={self.outputs}, "
            f"max_children={self.max_children})"
        )

    def __eq__(self, other) -> bool:
        if not isinstance(other, NodeConfig):
            return False
        return (
            self.inputs == other.inputs
            and self.outputs == other.outputs
            and self.max_children == other.max_children
        )

    def __ne__(self, other) -> bool:
        return not self == other

    def extend(self, other: "NodeConfig") -> Result[None, NodeConfigError]:
        """
        Extend the input and output dicts with values from `other`.

        Returns an error if if the two configs are incompatible,
        either due to duplicate io keys or mismatch in allowed number of child nodes.
        """
        if self.max_children != other.max_children:
            return Err(
                NodeConfigError(
                    f"Mismatch in max_children: {self.max_children} vs {other.max_children}"
                )
            )
        duplicate_inputs = []
        for key in other.inputs:
            if key in self.inputs:
                duplicate_inputs.append(key)
                continue
            self.inputs[key] = other.inputs[key]
        duplicate_outputs = []
        for key in other.outputs:
            if key in self.outputs:
                duplicate_outputs.append(key)
                continue
            self.outputs[key] = other.outputs[key]

        if duplicate_inputs or duplicate_outputs:
            msg = "Duplicate keys: "
            keys_strings = []
            if duplicate_inputs:
                keys_strings.append(f"inputs: {str(duplicate_inputs)}")
            if duplicate_outputs:
                keys_strings.append(f"outputs: {str(duplicate_outputs)}")
            msg += ", ".join(keys_strings)
            return Err(NodeConfigError(msg))
        return Ok(None)

    def copy(self) -> "NodeConfig":
        """
        Implement a custom copy operation that also updates all IO references.
        """
        return NodeConfig(
            inputs=deepcopy(self.inputs),
            outputs=deepcopy(self.outputs),
            max_children=self.max_children,
            tags=self.tags,
        )
