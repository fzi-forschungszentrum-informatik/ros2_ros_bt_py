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
from typing import Optional, Dict


from ros_bt_py.custom_types import RosTopicType
from ros_bt_py_interfaces.msg import Node as NodeMsg
from rclpy.node import Node

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"input_type": RosTopicType},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class MessageToFields(Leaf):
    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(MessageToFields, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self._message_type = self.options["input_type"].get_type_obj()

        node_inputs = {"in": self._message_type}
        node_outputs = {}

        self.passthrough = True

        if inspect.isclass(self._message_type):
            try:
                msg = self._message_type()
                for field in msg._fields_and_field_types:
                    node_outputs[field] = type(getattr(msg, field))
                self.passthrough = False
            except AttributeError:
                node_outputs["out"] = self._message_type
                self.logwarn(f"Non message type passed to: {self.name}")
        else:
            node_outputs["out"] = self._message_type

        self.node_config.extend(
            NodeConfig(options={}, inputs=node_inputs, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            for field in self.outputs:
                value = getattr(self.inputs["in"], field)
                if type(value) == tuple and self.outputs.get_type(field) == list:
                    self.outputs[field] = list(value)
                else:
                    self.outputs[field] = value
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"output_type": RosTopicType},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class FieldsToMessage(Leaf):
    """
    Take multiple fields as input and outputs a ROS message.

    The inputs will be named after the fields in the output message

    If the output is not as ROS message, the input will be be passed through

    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(FieldsToMessage, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self._message_type = self.options["output_type"].get_type_obj()

        node_inputs = {}
        node_outputs = {"out": self._message_type}

        self.passthrough = True

        if inspect.isclass(self._message_type):
            try:
                msg = self._message_type()
                for field in msg._fields_and_field_types:
                    node_inputs[field] = type(getattr(msg, field))
                self.passthrough = False
            except AttributeError:
                node_inputs["in"] = self._message_type
                self.logwarn(f"Non message type passed to: {self.name}")
        else:
            node_inputs["in"] = self._message_type

        self.node_config.extend(
            NodeConfig(options={}, inputs=node_inputs, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        if self.passthrough:
            self.outputs["out"] = self.inputs["in"]
        else:
            msg = self._message_type()

            for field in msg._fields_and_field_types:
                setattr(msg, field, self.inputs[field])

            self.outputs["out"] = msg
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
