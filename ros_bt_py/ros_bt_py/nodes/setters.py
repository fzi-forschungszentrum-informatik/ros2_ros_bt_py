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

from result import Result, Ok

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.helpers import rsetattr, BTNodeState


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"list_type": type},
        inputs={"list": list, "value": OptionRef("list_type")},
        outputs={"new_list": list},
        max_children=0,
    )
)
class AppendListItem(Leaf):
    """Appends `item` to the end of `list`."""

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.inputs.is_updated("list") or self.inputs.is_updated("value"):
            self.outputs["new_list"] = self.inputs["list"] + [self.inputs["value"]]

        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"object_type": type, "attr_name": str, "attr_type": type},
        inputs={
            "object": OptionRef("object_type"),
            "attr_value": OptionRef("attr_type"),
        },
        outputs={"new_object": OptionRef("object_type")},
        max_children=0,
    )
)
class SetAttr(Leaf):
    """Set the attribute named `attr` in `object`."""

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.inputs.is_updated("object") or self.inputs.is_updated("attr_value"):
            obj = deepcopy(self.inputs["object"])
            rsetattr(obj, self.options["attr_name"], self.inputs["attr_value"])
            self.outputs["new_object"] = obj
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"attr_name": str, "attr_type": type},
        inputs={"object": dict, "attr_value": OptionRef("attr_type")},
        outputs={"new_object": dict},
        max_children=0,
    )
)
class SetDictItem(Leaf):
    """Set the attribute named `attr` in `object`."""

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.inputs.is_updated("object") or self.inputs.is_updated("attr_value"):
            obj = deepcopy(self.inputs["object"])
            obj[self.options["attr_name"]] = self.inputs["attr_value"]
            self.outputs["new_object"] = obj
        return Ok(BTNodeState.SUCCEEDED)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        return Ok(BTNodeState.IDLE)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)
