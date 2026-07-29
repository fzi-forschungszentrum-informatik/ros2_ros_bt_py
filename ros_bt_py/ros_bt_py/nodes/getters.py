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
"""BT nodes to get values from containers and other nodes."""

import abc

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import (
    BoolType,
    GenericType,
    IntType,
    ListType,
    ReferenceDictType,
    ReferenceListType,
    ReferenceType,
    StringType,
    RosTopicType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState, rgetattr
from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={"succeed_on_stale_data": BoolType(allow_dynamic=False)},
        outputs={},
        max_children=1,
    )
)
class Getter(Decorator):

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.inputs.get_value_as("succeed_on_stale_data", bool):
            case Err(e):
                return Err(e)
            case Ok(b):
                self.succeed_on_stale_data = b
        for child in self.children:
            return child.setup()
        return Ok(BTNodeState.IDLE)

    @abc.abstractmethod
    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            return child.tick()
        return Ok(BTNodeState.SUCCEEDED)
        # Subclasses have to implement their own `_do_tick` methods to do the actual work

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.reset()
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            return child.untick()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "list_type": GenericType(),
            "index": IntType(min_value=0),
            "list": ReferenceListType(reference="list_type"),
        },
        outputs={"item": ReferenceType("list_type")},
        max_children=1,
    )
)
class GetListItem(Getter):
    """
    Extracts the item at the given `index` from `list`.

    The option parameter `succeed_on_stale_data` determines whether
    the node returns SUCCEEDED or RUNNING if `list` or `index` haven't been
    updated since the last tick.

    """

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        match super()._do_tick():
            case Err(e):
                return Err(e)
            case Ok(s):
                if s in [BTNodeState.FAILED, BTNodeState.RUNNING]:
                    return Ok(s)

        match self.inputs.any_updated("list", "index"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b

        try:
            match do(
                Ok(li[i])
                for li in self.inputs.get_value_as("list", list)
                for i in self.inputs.get_value_as("index", int)
            ):
                case Err(e):
                    return Err(e)
                case Ok(o):
                    out = o
        except IndexError as e:
            self.logerr(str(e))
            return Ok(BTNodeState.FAILED)

        if updated or self.succeed_on_stale_data:
            # Always set output when we return succeeded
            return self.outputs.set_value("item", out).map(
                lambda _: BTNodeState.SUCCEEDED
            )
        else:
            self.loginfo("No new data since last tick!")
            return Ok(BTNodeState.RUNNING)


@define_bt_node(
    NodeConfig(
        inputs={
            "value_type": GenericType(),
            "key": StringType(),
            "dict": ReferenceDictType(reference="value_type"),
        },
        outputs={"value": ReferenceType(reference="value_type")},
        max_children=1,
    )
)
class GetDictItem(Getter):
    """Get a item with a specific key from a dict input."""

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        match super()._do_tick():
            case Err(e):
                return Err(e)
            case Ok(s):
                if s in [BTNodeState.FAILED, BTNodeState.RUNNING]:
                    return Ok(s)

        match self.inputs.any_updated("dict", "key"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b

        try:
            match do(
                Ok(d[k])
                for d in self.inputs.get_value_as("dict", dict)
                for k in self.inputs.get_value_as("key", str)
            ):
                case Err(e):
                    return Err(e)
                case Ok(o):
                    out = o
        except KeyError as e:
            self.logwarn(str(e))
            return Ok(BTNodeState.FAILED)

        if updated or self.succeed_on_stale_data:
            # Always set output when returning succeeded
            return self.outputs.set_value("value", out).map(
                lambda _: BTNodeState.SUCCEEDED
            )
        else:
            self.loginfo("No new data since last tick!")
            return Ok(BTNodeState.RUNNING)


@define_bt_node(
    NodeConfig(
        inputs={
            "value_type": GenericType(),
            "keys": ListType(element_type=StringType()),
            "dict": ReferenceDictType(reference="value_type"),
        },
        outputs={"values": ReferenceListType(reference="value_type")},
        max_children=1,
    )
)
class GetMultipleDictItems(Getter):
    """Get multiple dict items with a specific list of keys."""

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        match super()._do_tick():
            case Err(e):
                return Err(e)
            case Ok(s):
                if s in [BTNodeState.FAILED, BTNodeState.RUNNING]:
                    return Ok(s)

        match self.inputs.any_updated("dict", "keys"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b

        try:
            match do(
                Ok([d[k] for k in li])
                for d in self.inputs.get_value_as("dict", dict)
                for li in self.inputs.get_value_as("keys", list)
            ):
                case Err(e):
                    return Err(e)
                case Ok(o):
                    out_list = o
        except KeyError as e:
            self.logwarn(str(e))
            return Ok(BTNodeState.FAILED)

        if updated or self.succeed_on_stale_data:
            return self.outputs.set_value("values", out_list).map(
                lambda _: BTNodeState.SUCCEEDED
            )
        else:
            self.loginfo("No new data since last tick!")
            return Ok(BTNodeState.RUNNING)


@define_bt_node(
    NodeConfig(
        inputs={
            "object_type": RosTopicType(),
            "attr_type": GenericType(),
            "attr_name": StringType(),
            "object": ReferenceType(reference="object_type"),
        },
        outputs={"attr": ReferenceType(reference="attr_type")},
        max_children=1,
    )
)
class GetAttr(Getter):
    """
    Get a specific attribute form a python object.

    This can also be done with nested attributes,
    e.g. the sec argument the timestamp of a
    std_msgs/msg/Header.msg can be extracted by using stamp.sec

    """

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        match super()._do_tick():
            case Err(e):
                return Err(e)
            case Ok(s):
                if s in [BTNodeState.FAILED, BTNodeState.RUNNING]:
                    return Ok(s)

        match self.inputs.any_updated("object", "attr_name"):
            case Err(e):
                return Err(e)
            case Ok(b):
                updated = b

        try:
            # TODO Maybe it would be nice to allow for calling 0-argument functions this way?
            match do(
                Ok(rgetattr(o, a))
                for o in self.inputs.get_value_as("object", object)
                for a in self.inputs.get_value_as("attr_name", str)
            ):
                case Err(e):
                    return Err(e)
                case Ok(o):
                    out = o
        except AttributeError as e:
            self.logwarn(str(e))
            return Ok(BTNodeState.FAILED)

        if updated or self.succeed_on_stale_data:
            return self.outputs.set_value("attr", out).map(
                lambda _: BTNodeState.SUCCEEDED
            )
        else:
            self.loginfo("No new data since last tick!")
            return Ok(BTNodeState.RUNNING)
