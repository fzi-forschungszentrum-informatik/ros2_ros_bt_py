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
from rclpy.node import Node

from ros_bt_py_interfaces.msg import NodeState
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

    def __init__(  # noqa: C901
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        """Create the tree manager, load the subtree."""
        super().__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )
        if not self.has_ros_node:
            raise BehaviorTreeException(
                "{self.name} does not have a reference to a ROS Node!"
            )

        self.root: Optional[BTNode] = None
        # since the subtree gets a prefix, we can just have it use the
        # parent debug manager
        self.manager: TreeManager = TreeManager(
            ros_node=self.ros_node,
            name=name,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
        )
        self.load_subtree()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_structure(
                self.name, self.manager.structure_to_msg()
            )

    def load_subtree(self) -> None:
        response = LoadTree.Response()
        response = self.manager.load_tree(
            request=LoadTree.Request(
                tree=TreeStructure(
                    path=self.options["subtree_path"].path,
                    name=self.name,
                )
            ),
            response=response,
            prefix=self.name,
        )

        if not get_success(response):
            self.logwarn(
                f"Failed to load subtree {self.name}: {get_error_message(response)}"
            )
            self.state = NodeState.BROKEN

            self.outputs["load_success"] = False
            self.outputs["load_error_msg"] = get_error_message(response)
            return
        else:
            self.outputs["load_success"] = True

        # If we loaded the tree successfully, change node_config to
        # include the public inputs and outputs
        subtree_inputs: Dict[str, type] = {}
        subtree_outputs: Dict[str, type] = {}

        # If io nodes are used restrict the subtrees inputs and outputs to io nodes
        io_inputs: List[str] = []
        io_outputs: List[str] = []
        self._find_inputs_and_output(
            io_inputs=io_inputs,
            io_outputs=io_outputs,
            subtree_inputs=subtree_inputs,
            subtree_outputs=subtree_outputs,
        )

        # merge subtree input and option dicts, so we can receive
        # option updates between ticks
        # TODO: Use result type.
        self.node_config.extend(
            NodeConfig(
                options={},
                inputs=subtree_inputs,
                outputs=subtree_outputs,
                max_children=0,
            )
        )
        self._register_data_forwarding(
            io_inputs=io_inputs,
            io_outputs=io_outputs,
            subtree_inputs=subtree_inputs,
            subtree_outputs=subtree_outputs,
        )

    def _find_inputs_and_output(
        self,
        io_inputs: List,
        io_outputs: List,
        subtree_inputs: Dict,
        subtree_outputs: Dict,
    ) -> None:
        subtree_msg = self.manager.structure_to_msg()
        if self.options["use_io_nodes"]:
            for node in subtree_msg.nodes:
                if node.module == "ros_bt_py.nodes.io":
                    if (
                        node.node_class == "IOInput"
                        or node.node_class == "IOInputOption"
                    ):
                        io_inputs.append(node.name)
                    elif (
                        node.node_class == "IOOutput"
                        or node.node_class == "IOOutputOption"
                    ):
                        io_outputs.append(node.name)
            modified_public_node_data = []
            for node_data in subtree_msg.public_node_data:
                if node_data.data_kind == "inputs" and node_data.node_name in io_inputs:
                    modified_public_node_data.append(node_data)
                elif (
                    node_data.data_kind == "outputs"
                    and node_data.node_name in io_outputs
                ):
                    modified_public_node_data.append(node_data)
            subtree_msg.public_node_data = modified_public_node_data

        for node_data in subtree_msg.public_node_data:
            # Remove the prefix from the node name to make for nicer
            # input/output names (and also not break wirings)
            node_name = node_data.node_name
            if node_name.startswith(self.name):
                node_name = node_name[len(self.name) + 1 :]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                subtree_inputs[f"{node_name}.{node_data.data_key}"] = (
                    self.manager.nodes[node_data.node_name].inputs.get_type(
                        node_data.data_key
                    )
                )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                subtree_outputs[f"{node_name}.{node_data.data_key}"] = (
                    self.manager.nodes[node_data.node_name].outputs.get_type(
                        node_data.data_key
                    )
                )

    def _register_data_forwarding(
        self,
        io_inputs: List,
        io_outputs: List,
        subtree_inputs: Dict,
        subtree_outputs: Dict,
    ) -> None:
        # Register the input and output values from the subtree
        self._register_node_data(source_map=subtree_inputs, target_map=self.inputs)
        self._register_node_data(source_map=subtree_outputs, target_map=self.outputs)

        # Handle forwarding inputs and outputs using the subscribe mechanics:
        for node_data in self.manager.structure_to_msg().public_node_data:
            # get the node name without prefix to match our renamed
            # inputs and outputs
            node_name = node_data.node_name
            if node_name.startswith(self.name):
                node_name = node_name[len(self.name) + 1 :]

            if node_data.data_kind == NodeDataLocation.INPUT_DATA:
                if (
                    self.options["use_io_nodes"]
                    and node_data.node_name not in io_inputs
                ):
                    self.logdebug(
                        f"removed an unconnected input ({node_name}) from the subtree"
                    )
                else:
                    self.inputs.subscribe(
                        key=f"{node_name}.{node_data.data_key}",
                        callback=self.manager.nodes[
                            node_data.node_name
                        ].inputs.get_callback(node_data.data_key),
                    )
            elif node_data.data_kind == NodeDataLocation.OUTPUT_DATA:
                if (
                    self.options["use_io_nodes"]
                    and node_data.node_name not in io_outputs
                ):
                    pass
                else:
                    self.manager.nodes[node_data.node_name].outputs.subscribe(
                        key=node_data.data_key,
                        callback=self.outputs.get_callback(
                            f"{node_name}.{node_data.data_key}"
                        ),
                    )

    def _do_setup(self):
        # FIXME: This does not use the result!
        self.root = self.manager.find_root()
        if self.root is None:
            raise BehaviorTreeException(
                "Cannot find root in subtree, does the subtree "
                f"{self.options['subtree_path']} exist?"
            )
        self.root.setup()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_state(
                self.name, self.manager.state_to_msg()
            )

    def _do_tick(self):
        if not self.root:
            return NodeState.BROKEN
        new_state = self.root.tick()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_state(
                self.name, self.manager.state_to_msg()
            )
            if self.subtree_manager.get_publish_data():
                self.subtree_manager.add_subtree_data(
                    self.name, self.manager.data_to_msg()
                )
        return new_state

    def _do_untick(self):
        if not self.root:
            return NodeState.BROKEN
        new_state = self.root.untick()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_state(
                self.name, self.manager.state_to_msg()
            )
        return new_state

    def _do_reset(self):
        if not self.root:
            return NodeState.IDLE
        new_state = self.root.reset()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_state(
                self.name, self.manager.state_to_msg()
            )
        return new_state

    def _do_shutdown(self):
        if not self.root:
            return NodeState.SHUTDOWN
        self.root.shutdown()
        if self.subtree_manager:
            # FIXME: This does not use the result!
            self.subtree_manager.add_subtree_state(
                self.name, self.manager.state_to_msg()
            )

    def _do_calculate_utility(self):
        # FIXME: This does not use the result!
        self.root = self.manager.find_root()
        if self.root is not None:
            return self.root.calculate_utility()
        else:
            return UtilityBounds(
                has_lower_bound_success=False,
                has_upper_bound_success=False,
                has_lower_bound_failure=False,
                has_upper_bound_failure=False,
            )
