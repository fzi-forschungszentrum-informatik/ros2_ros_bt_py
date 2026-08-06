# Copyright (c) 2026 FZI Forschungszentrum Informatik
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
#    * Neither the name of the copyright holder nor the names of its
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

"""BT node to encapsulate a part of a tree in a reusable subtree."""

from typing import Optional
import uuid

from ros_bt_py.nodes.io import IOInput, IOOutput
from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py_interfaces.msg import (
    UtilityBounds,
    TreeStructure,
)
from ros_bt_py_interfaces.srv import LoadTree

from ros_bt_py.data_flow_manager import DataFlowManager
from ros_bt_py.logging_manager import LoggingManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.tree_exec_manager import TreeExecManager
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node import Node as BTNode
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.data_types import DataContainer, PathType
from ros_bt_py.helpers import BTNodeState


# Type alias for ros uuids
ROS_UUID = str


@define_bt_node(
    NodeConfig(
        inputs={
            "subtree_path": PathType(allow_dynamic=False),
        },
        outputs={},
        max_children=0,
    )
)
class Subtree(Leaf):
    """
    Load a subtree from the location pointed to by `subtree_uri`.

    This is the only node that modifies its `node_config` member - it
    will populate its inputs and outputs when its constructor is
    called, based on the public inputs and outputs of the subtree.

    Please note that it is **NOT** possible to have public *option*
    values. Since they can affect the types of inputs/outputs, they
    could only feasibly be set in the Subtree node's own options, but
    at that point we don't know their names or types yet.
    """

    manager: TreeExecManager
    is_set_up = False

    def setup_node(self) -> Result[None, NodeConfigError]:
        if not self.has_ros_node:
            raise BehaviorTreeException(
                "{self.name} does not have a reference to a ROS Node!"
            )

        self.root: Optional[BTNode] = None

        # Since node_id is only unique within one tree and tree_id has to be globally unique
        #   we generate a new uuid and store it in `tree_ref`
        self.tree_ref = uuid.uuid4()

        # since the subtree gets a prefix, we can just have it use the
        # parent debug manager TODO Is that still true?
        self.nested_subtree_manager = SubtreeManager()
        subtree_logging_manager = (
            LoggingManager(
                ros_node=self.ros_node,
                publish_log_callback=self.logging_manager.publish_log_callback,
            )
            if self.logging_manager is not None
            else None
        )
        subtree_data_flow_manager = DataFlowManager(
            incoming_data=self.node_config.inputs,
            outgoing_data=self.node_config.outputs,
        )
        self.manager = TreeExecManager(
            ros_node=self.ros_node,
            name=self.name,
            tree_id=self.tree_ref,
            debug_manager=self.debug_manager,
            subtree_manager=self.nested_subtree_manager,
            logging_manager=subtree_logging_manager,
            data_flow_manager=subtree_data_flow_manager,
        )

        match self.load_subtree():
            case Err(e):
                return Err(e)
            case Ok(None):
                pass

        if self.subtree_manager:
            self.subtree_manager.add_subtree_structure(
                self.node_id, self.manager.structure_to_msg()
            )
            self.subtree_manager.add_nested_manager(
                self.node_id, self.nested_subtree_manager
            )

        return Ok(None)

    def load_subtree(self) -> Result[None, NodeConfigError]:
        match self.inputs.get_value_as("subtree_path", str):
            case Err(e):
                return Err(e)
            case Ok(s):
                subtree_path = s
        response = self.manager.load_tree(
            request=LoadTree.Request(
                tree=TreeStructure(
                    path=subtree_path,
                    name=self.name,
                )
            ),
            response=LoadTree.Response(),
        )

        if not response.success:
            error_msg = f"Failed to load subtree {self.name}: {response.error_message}"
            self.logwarn(error_msg)
            return Err(NodeConfigError(error_msg))

        return Ok(None)

    def add_extra_inputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        # Only load the subtree once
        if not self.is_set_up:
            match self.setup_node():
                case Err(e):
                    return Err(e)
                case Ok(None):
                    self.is_set_up = True
        node_inputs = {}
        for node in self.manager.nodes.values():
            if not isinstance(node, IOInput):
                continue
            node_inputs[f"{str(node.node_id)}.in"] = node.node_config.inputs[
                "in"
            ].get_runtime_type()
        return Ok(node_inputs)

    def add_extra_outputs(self) -> Result[dict[str, DataContainer], NodeConfigError]:
        # Only load the subtree once
        if not self.is_set_up:
            match self.setup_node():
                case Err(e):
                    return Err(e)
                case Ok(None):
                    self.is_set_up = True
        node_outputs = {}
        for node in self.manager.nodes.values():
            if not isinstance(node, IOOutput):
                continue
            node_outputs[f"{str(node.node_id)}.out"] = node.node_config.outputs[
                "out"
            ].get_runtime_type()
        return Ok(node_outputs)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        match self.manager.find_root():
            case Err(e):
                return Err(e)
            case Ok(n):
                self.root = n
        if self.root is None:
            return Ok(BTNodeState.IDLE)

        match self.manager.data_flow_manager.initialize(
            self.manager.nodes, self.manager.wirings
        ):
            case Err(e):
                return Err(BehaviorTreeException(e))
            case Ok(None):
                pass
        setup_root_result = self.root.setup()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
        return setup_root_result

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.root:
            return Err(BehaviorTreeException("Subtree has no root node"))

        match self.manager.data_flow_manager.push_incoming_data():
            case Err(e):
                return Err(BehaviorTreeException(e))
            case Ok(None):
                pass
        tick_root_result = self.root.tick()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
            if self.subtree_manager.get_publish_data():
                self.subtree_manager.add_subtree_data(
                    self.node_id, self.manager.data_to_msg()
                )
        return tick_root_result

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.root:
            return Err(BehaviorTreeException("Subtree has no root node"))
        untick_root_result = self.root.untick()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
        return untick_root_result

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.root:
            return Ok(BTNodeState.IDLE)
        reset_root_result = self.root.reset()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
        return reset_root_result

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.root:
            return Ok(BTNodeState.SHUTDOWN)
        shutdown_root_result = self.root.shutdown()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
        return shutdown_root_result

    def _do_calculate_utility(self) -> Result[UtilityBounds, BehaviorTreeException]:
        find_root_result = self.manager.find_root()
        if find_root_result.is_err():
            return Ok(
                UtilityBounds(
                    has_lower_bound_success=False,
                    has_upper_bound_success=False,
                    has_lower_bound_failure=False,
                    has_upper_bound_failure=False,
                )
            )
        self.root = find_root_result.unwrap()
        if self.root is None:
            return Ok(UtilityBounds(can_execute=False))
        return self.root.calculate_utility()
