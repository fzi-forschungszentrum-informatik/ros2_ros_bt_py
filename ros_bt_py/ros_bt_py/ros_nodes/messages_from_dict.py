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
"""BT nodes to convert Dicts to ROS messages of a specific type."""

from typing import Dict, Optional

from rclpy.node import Node
from rosidl_runtime_py.set_message import set_message_fields

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.custom_types import RosTopicType
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager

from ros_bt_py_interfaces.msg import Node as NodeMsg



@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"message_type": RosTopicType},
        inputs={"dict": dict},
        outputs={},
        max_children=0,
    )
)
class MessageFromDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

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
        super(MessageFromDict, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self._message_type = self.options["message_type"].get_type_obj()

        node_outputs = {"message": self._message_type}

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated("dict") or self.state == NodeMsg.IDLE:
            message = self._message_type()
            try:
                set_message_fields(
                    message,
                    self.inputs["dict"],
                )
                self.outputs["message"] = message
            except (
                AttributeError,
                TypeError,
            ) as ex:
                self.logerr(
                    f"Error populating message of type {self._message_type.__name__}: "
                    f"{str(ex)}"
                )
                return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"message_type": RosTopicType, "dict": dict},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class MessageFromConstDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

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
        super(MessageFromConstDict, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        self._message_type = self.options["message_type"].get_type_obj()

        node_outputs = {"message": self._message_type}

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        pass

    def _do_tick(self):
        message = self._message_type()
        try:
            set_message_fields(
                message,
                self.options["dict"],
            )
            self.outputs["message"] = message
        except (TypeError, AttributeError) as ex:
            self.logerr(
                f"Error populating message of type {self._message_type.__name__}: "
                f"{str(ex)}"
            )
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
