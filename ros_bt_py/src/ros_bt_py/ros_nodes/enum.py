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
from typing import Optional, Dict
from rclpy.node import Node

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.ros_helpers import EnumValue, get_message_constant_fields


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"ros_message_type": type, "constant_name": EnumValue},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class Enum(Leaf):
    """Expose a constant in a ROS message as an output."""

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(Enum, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_outputs = {}

        constants = get_message_constant_fields(self.options["ros_message_type"])

        if not constants:
            raise BehaviorTreeException(
                f"{self.options['ros_message_type']} has no constant fields"
            )

        if self.options["constant_name"].enum_value not in constants:
            raise BehaviorTreeException(
                "{self.options['ros_message_type']} has no field {self.options['constant_name']}"
            )

        self.msg = self.options["ros_message_type"]()

        node_outputs["out"] = type(
            getattr(self.msg, self.options["constant_name"].enum_value)
        )

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        self.outputs["out"] = getattr(
            self.msg, self.options["constant_name"].enum_value
        )
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
        options={"ros_message_type": type},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class EnumFields(Leaf):
    """
    Expose the constants in a ROS message as multiple output fields.

    The outputs will be named after the fields in the ROS message
    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(EnumFields, self).__init__(
            options=options,
            debug_manager=debug_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )

        node_outputs = {}

        constants = get_message_constant_fields(self.options["ros_message_type"])

        if not constants:
            raise BehaviorTreeException(
                f"{self.options['ros_message_type']} has no constant fields"
            )

        self.msg = self.options["ros_message_type"]()

        for field in constants:
            node_outputs[field] = type(getattr(self.msg, field))

        self.node_config.extend(
            NodeConfig(options={}, inputs={}, outputs=node_outputs, max_children=0)
        )

        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        return NodeMsg.IDLE

    def _do_tick(self):
        for field in self.outputs:
            self.outputs[field] = getattr(self.msg, field)
        return NodeMsg.SUCCEEDED

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE
