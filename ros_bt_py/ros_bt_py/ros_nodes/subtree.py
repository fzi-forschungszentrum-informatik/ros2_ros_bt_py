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
"""BT node to encapsulate a part of a tree in a reusable subtree."""
from typing import List, Optional, Dict

from result import Result, Ok, Err
import uuid

from rclpy.node import Node

from ros_bt_py.logging_manager import LoggingManager
from ros_bt_py_interfaces.msg._node_structure import NodeStructure

from ros_bt_py_interfaces.msg import UtilityBounds, TreeStructure, NodeDataLocation
from ros_bt_py_interfaces.srv import LoadTree

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.tree_manager import TreeManager, get_success, get_error_message
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node import Node as BTNode
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.custom_types import FilePath
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros


# Type alias for ros uuids
ROS_UUID = str


@define_bt_node(
    NodeConfig(
        version="0.2.0",
        options={"subtree_path": FilePath, "use_io_nodes": bool},
        inputs={},
        outputs={"load_success": bool, "load_error_msg": str},
        max_children=0,
        optional_options=["use_io_nodes"],
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

    manager: TreeManager

    def __init__(self, *args, **kwargs) -> None:
        """Create the tree manager, load the subtree."""
        super().__init__(*args, **kwargs)
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
        self.subtree_logging_manager = (
            LoggingManager(
                ros_node=self.ros_node,
                publish_log_callback=self.logging_manager.publish_log_callback,
            )
            if self.logging_manager is not None
            else None
        )
        self.manager: TreeManager = TreeManager(
            ros_node=self.ros_node,
            name=self.name,
            tree_id=self.tree_ref,
            debug_manager=self.debug_manager,
            subtree_manager=self.nested_subtree_manager,
            logging_manager=self.subtree_logging_manager,
        )

        match self.load_subtree():
            case Err(e):
                raise e
            case Ok(None):
                pass

        if self.subtree_manager:
            self.subtree_manager.add_subtree_structure(
                self.node_id, self.manager.structure_to_msg()
            )
            self.subtree_manager.add_nested_manager(
                self.node_id, self.nested_subtree_manager
            )

    def load_subtree(self) -> Result[None, BehaviorTreeException]:
        response = LoadTree.Response()
        response = self.manager.load_tree(
            request=LoadTree.Request(
                tree=TreeStructure(
                    path=self.options["subtree_path"].path,
                    name=self.name,
                )
            ),
            response=response,
        )

        if not get_success(response):
            self.logwarn(
                f"Failed to load subtree {self.name}: {get_error_message(response)}"
            )
            # TODO Should this be flagged as broken, since we convey load failure as an output
            #   Suggesting that it is intended behaviour and not an error.
            self.state = BTNodeState.BROKEN

            self.outputs["load_success"] = False
            self.outputs["load_error_msg"] = get_error_message(response)
            return Ok(None)
        else:
            self.outputs["load_success"] = True

        # If we loaded the tree successfully, change node_config to
        # include the public inputs and outputs
        subtree_inputs: dict[str, type] = {}
        subtree_outputs: dict[str, type] = {}

        # If io nodes are used restrict the subtrees inputs and outputs to io nodes
        io_inputs: list[ROS_UUID] = []
        io_outputs: list[ROS_UUID] = []
        self._find_inputs_and_output(
            io_inputs=io_inputs,
            io_outputs=io_outputs,
            subtree_inputs=subtree_inputs,
            subtree_outputs=subtree_outputs,
        )

        # merge subtree input and option dicts, so we can receive
        # option updates between ticks
        extend_result = self.node_config.extend(
            NodeConfig(
                options={},
                inputs=subtree_inputs,
                outputs=subtree_outputs,
                max_children=0,
            )
        )
        if extend_result.is_err():
            return extend_result

        register_result = self._register_data_forwarding(
            io_inputs=io_inputs,
            io_outputs=io_outputs,
            subtree_inputs=subtree_inputs,
            subtree_outputs=subtree_outputs,
        )
        if register_result.is_err():
            return register_result

        return Ok(None)

    def _find_inputs_and_output(
        self,
        io_inputs: list[ROS_UUID],
        io_outputs: list[ROS_UUID],
        subtree_inputs: dict[str, type],
        subtree_outputs: dict[str, type],
    ) -> None:
        node_data: NodeDataLocation
        subtree_msg = self.manager.structure_to_msg()
        if self.options["use_io_nodes"]:
            node_msg: NodeStructure
            for node_msg in subtree_msg.nodes:
                if node_msg.module == "ros_bt_py.nodes.io":
                    if (
                        node_msg.node_class == "IOInput"
                        or node_msg.node_class == "IOInputOption"
                    ):
                        io_inputs.append(node_msg.node_id)
                    elif (
                        node_msg.node_class == "IOOutput"
                        or node_msg.node_class == "IOOutputOption"
                    ):
                        io_outputs.append(node_msg.node_id)
            modified_public_node_data = []
            for node_data in subtree_msg.public_node_data:
                if (
                    node_data.data_kind == NodeDataLocation.INPUT_DATA
                    and node_data.node_id in io_inputs
                ):
                    modified_public_node_data.append(node_data)
                elif (
                    node_data.data_kind == NodeDataLocation.OUTPUT_DATA
                    and node_data.node_id in io_outputs
                ):
                    modified_public_node_data.append(node_data)
            subtree_msg.public_node_data = modified_public_node_data

        for node_data in subtree_msg.public_node_data:
            # Since those are internal ids, we assume them to be safe
            node = self.manager.nodes[ros_to_uuid(node_data.node_id).unwrap()]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                subtree_inputs[f"{node.name}.{node_data.data_key}"] = (
                    node.inputs.get_type(node_data.data_key)
                )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                subtree_outputs[f"{node.name}.{node_data.data_key}"] = (
                    node.outputs.get_type(node_data.data_key)
                )

    def _register_data_forwarding(
        self,
        io_inputs: list[ROS_UUID],
        io_outputs: list[ROS_UUID],
        subtree_inputs: dict[str, type],
        subtree_outputs: dict[str, type],
    ) -> Result[None, BehaviorTreeException]:
        # Register the input and output values from the subtree
        register_result = self._register_node_data(
            source_map=subtree_inputs, target_map=self.inputs
        )
        if register_result.is_err():
            return register_result
        register_result = self._register_node_data(
            source_map=subtree_outputs, target_map=self.outputs
        )
        if register_result.is_err():
            return register_result

        # Handle forwarding inputs and outputs using the subscribe mechanics:
        node_data: NodeDataLocation
        for node_data in self.manager.structure_to_msg().public_node_data:
            # Since those are internal ids, we assume them to be safe
            node = self.manager.nodes[ros_to_uuid(node_data.node_id).unwrap()]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                if self.options["use_io_nodes"] and node_data.node_id not in io_inputs:
                    self.logdebug(
                        f"removed an unconnected input ({node.name}) from the subtree"
                    )
                else:
                    self.inputs.subscribe(
                        key=f"{node.name}.{node_data.data_key}",
                        callback=node.inputs.get_callback(node_data.data_key),
                    )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                if self.options["use_io_nodes"] and node_data.node_id not in io_outputs:
                    pass
                else:
                    node.outputs.subscribe(
                        key=node_data.data_key,
                        callback=self.outputs.get_callback(
                            f"{node.name}.{node_data.data_key}"
                        ),
                    )
        return Ok(None)

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        find_root_result = self.manager.find_root()
        if find_root_result.is_err():
            return Err(find_root_result.unwrap_err())
        self.root = find_root_result.unwrap()
        if self.root is None:
            return Ok(BTNodeState.IDLE)
        setup_root_result = self.root.setup()
        if self.subtree_manager:
            self.subtree_manager.add_subtree_state(
                self.node_id, self.manager.state_to_msg()
            )
        return setup_root_result

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if not self.root:
            # TODO Should this be an Err() ??? same also above in setup
            return Ok(BTNodeState.BROKEN)
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
            # TODO Should this be an Err() ???
            return Ok(BTNodeState.BROKEN)
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
