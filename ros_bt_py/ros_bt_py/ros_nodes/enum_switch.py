# Copyright 2025 FZI Forschungszentrum Informatik
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

from rclpy.node import Node

from result import Result, Ok, Err, is_err

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.custom_types import TypeWrapper, RosTopicType, ROS_TYPE_FULL
from ros_bt_py.ros_helpers import EnumValue, get_message_constant_fields


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"ros_message_type": TypeWrapper(RosTopicType, info=ROS_TYPE_FULL)},
        inputs={},
        outputs={},
        max_children=None,
    )
)
class EnumSwitch(FlowControl):
    def __init__(
        self,
        options: dict | None = None,
        debug_manager: DebugManager | None = None,
        subtree_manager: SubtreeManager | None = None,
        name: str | None = None,
        ros_node: Node | None = None,
    ):
        super(EnumSwitch, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
        )

        self._message_class = self.options["ros_message_type"].get_type_obj()

        self.possible_children = get_message_constant_fields(self._message_class)

        # Raise Error if no possible children
        if not self.possible_children:
            raise ValueError(f"{self._message_class} has no constant fields")

        # Check if all constants have equal type and raise error if not. Otherwise set input to
        # constant type
        pchild_types = [
            type(getattr(self._message_class, field))
            for field in self.possible_children
        ]
        if len(set(pchild_types)) > 1:
            raise ValueError(
                f"{self._message_class} contains constant fields of multiple types"
            )

        node_inputs = {"case": pchild_types[0]}

        extend_result = self.node_config.extend(
            NodeConfig(options={}, inputs=node_inputs, outputs={}, max_children=None)
        )
        if extend_result.is_err():
            raise extend_result.unwrap_err()

        register_result = self._register_node_data(
            source_map=node_inputs, target_map=self.inputs
        )

        if register_result.is_err():
            raise register_result.unwrap_err()

        # Create possible child dict

        self.msg = self._message_class()

        self.pchild_dict = dict(
            zip(
                [getattr(self.msg, entry) for entry in self.possible_children],
                self.possible_children,
            )
        )

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.child_map = {child.name.split(".")[-1]: child for child in self.children}
        for child in self.children:
            if child.name.split(".")[-1] not in self.possible_children:
                self.logerr("Unwanted child detected please fix name")
            match child.setup():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        name = self.inputs["case"]

        if name in self.pchild_dict.keys():
            e_name = self.pchild_dict[name]
        else:
            self.logwarn("Input did not match possible children.")
            return Ok(BTNodeState.FAILED)

        if e_name not in self.child_map:
            self.logwarn(
                "Ticking without matching child. Is this really what you want?"
            )
            return Ok(BTNodeState.FAILED)

        # If we've previously succeeded or failed, untick all children
        if self.state in [BTNodeState.SUCCEEDED, BTNodeState.FAILED]:
            for child in self.children:
                match child.reset():
                    case Err(e):
                        return Err(e)
                    case Ok(_):
                        pass

        for child_name, child in self.child_map.items():
            if not child_name == e_name:
                match child.untick():
                    case Err(e):
                        return Err(e)
                    case Ok(_):
                        pass

        return self.child_map[e_name].tick()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.untick():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        for child in self.children:
            match child.reset():
                case Err(e):
                    return Err(e)
                case Ok(_):
                    pass
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)
