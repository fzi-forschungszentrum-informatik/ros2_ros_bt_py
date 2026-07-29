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

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import (
    GenericType,
    ReferenceListType,
    ReferenceType,
    ReferenceDictType,
    StringType,
    RosTopicType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import rsetattr, BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@define_bt_node(
    NodeConfig(
        inputs={
            "list_type": GenericType(),
            "list": ReferenceListType(reference="list_type"),
            "value": ReferenceType(reference="list_type"),
        },
        outputs={"new_list": ReferenceListType(reference="list_type")},
        max_children=0,
    )
)
class AppendListItem(Leaf):
    """Appends `item` to the end of `list`."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            do(
                Ok(li + [v])
                for li in self.inputs.get_value_as("list", list)
                for v in self.inputs.get_value("value")
            )
            .and_then(lambda val: self.outputs.set_value("new_list", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "object_type": RosTopicType(),
            "attr_type": GenericType(),
            "attr_name": StringType(),
            "object": ReferenceType(reference="object_type"),
            "attr_value": ReferenceType(reference="attr_type"),
        },
        outputs={"new_object": ReferenceType(reference="object_type")},
        max_children=0,
    )
)
class SetAttr(Leaf):
    """Set the attribute named `attr` in `object`."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        def set_and_return(obj, attr, val):
            rsetattr(obj, attr, val)
            return obj

        return (
            do(
                Ok(set_and_return(o, n, v))
                for o in self.inputs.get_value("object")
                for n in self.inputs.get_value_as("attr_name", str)
                for v in self.inputs.get_value("value")
            )
            .and_then(lambda val: self.outputs.set_value("new_object", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "attr_type": GenericType(),
            "attr_name": StringType(),
            "object": ReferenceDictType(reference="attr_type"),
            "attr_value": ReferenceType(reference="attr_type"),
        },
        outputs={"new_object": ReferenceDictType(reference="attr_type")},
        max_children=0,
    )
)
class SetDictItem(Leaf):
    """Set the attribute named `attr` in `object`."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        def set_and_return(dir, key, val):
            dir[key] = val
            return dir

        return (
            do(
                Ok(set_and_return(d, k, v))
                for d in self.inputs.get_value_as("object", dict)
                for k in self.inputs.get_value_as("attr_name", str)
                for v in self.inputs.get_value("attr_value")
            )
            .and_then(lambda val: self.outputs.set_value("new_object", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
