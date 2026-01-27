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
from importlib import metadata
import inspect
import os
import uuid
from copy import deepcopy
from functools import wraps
from packaging.version import Version
from threading import Thread, Lock, RLock
from typing import Any, Callable, Dict, Optional, List, cast

from result import Err, Ok, Result

import rclpy
import rclpy.logging
from rclpy.utilities import ok
import rclpy.node
from rclpy.duration import Duration
from rclpy.logging import get_logger

import yaml
import yaml.scanner
from typeguard import typechecked


from ros_bt_py.migrate_tree_files import migrate_legacy_tree_structure
from ros_bt_py_interfaces.msg import (
    DocumentedNode,
    NodeStructure,
    NodeState,
    NodeIO,
    NodeOption,
    NodeDataLocation,
    TreeStructure,
    TreeStructureList,
    TreeState,
    TreeStateList,
    TreeData,
    TreeDataList,
)

from ros_bt_py_interfaces.srv import (
    AddNode,
    AddNodeAtIndex,
    ChangeTreeName,
    ClearTree,
    ControlTreeExecution,
    GenerateSubtree,
    GetAvailableNodes,
    GetSubtree,
    LoadTree,
    LoadTreeFromPath,
    MigrateTree,
    MorphNode,
    MoveNode,
    ReloadTree,
    RemoveNode,
    ReplaceNode,
    SetOptions,
    SetSimulateTick,
    WireNodeData,
)

import rosidl_runtime_py
from std_srvs.srv import SetBool

import ament_index_python

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import (
    BehaviorTreeException,
    MissingParentError,
    TreeTopologyError,
)
from ros_bt_py.helpers import (
    BTNodeState,
    json_encode,
    json_decode,
)
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros
from ros_bt_py.node import Node, load_node_module, increment_name
from ros_bt_py.node_config import OptionRef

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float64

from rclpy_message_converter import message_converter


def is_edit_service(func):
    """
    Decorate tree editing service handlers to prohibit them from editing while the active tree.

    This allows the common behavior of responding with a response that
    has success=False and an error_message if the tree is not
    currently editable, relying on all editing service responses to
    have at least those two members.

    It also ensures that all edits are atomic, i.e. external service
    calls cannot interweave. The lock used to ensure this is a
    `threading.RLock`, which means the service handlers *can* call
    each other if need be.
    """

    @wraps(func)
    def service_handler(self: "TreeManager", request: Any, response: Any, **kwds):
        tree_state = self.state
        if tree_state != TreeState.EDITABLE:
            response.success = False
            response.error_message = (
                f"Cannot edit tree in state {tree_state}."
                f"You need to shut down the tree to enable editing."
            )
            return response
        with self._edit_lock:
            return func(self, request, response, **kwds)

    return service_handler


@typechecked
def parse_tree_yaml(tree_yaml: str) -> MigrateTree.Response:
    response = MigrateTree.Response()

    data = yaml.safe_load_all(tree_yaml)
    read_data = False
    for datum in data:
        if datum is None:
            continue
        if not read_data:
            if Version(datum.get("version", "0.0.0")) < Version(
                metadata.version("ros_bt_py")
            ):
                match migrate_legacy_tree_structure(datum):
                    case Err(e):
                        response.success = False
                        response.error_message = (
                            f"Failed to migrate legacy tree file: {e}"
                        )
                        return response
                    case Ok(d):
                        datum = d
            datum.pop("version")
            rosidl_runtime_py.set_message_fields(response.tree, datum)
            read_data = True
        else:
            response.success = False
            response.error_message = (
                "Tree YAML file must contain exactly one YAML object!"
            )
            return response
    if not read_data:
        response.success = False
        response.error_message = "No data in YAML file!"

        return response

    response.success = True
    return response


@typechecked
def load_tree_from_file(
    request: MigrateTree.Request, response: MigrateTree.Response
) -> MigrateTree.Response:
    """Load a tree file from disk."""
    tree = request.tree
    while not tree.nodes:
        file_path = ""
        if not tree.path:
            response.success = False
            response.error_message = (
                "Trying to load tree, but found no nodes and "
                f"no path to read from: {str(tree)}"
            )
            return response

        if tree.path.startswith("file://"):
            file_path = tree.path[len("file://") :]

        elif tree.path.startswith("package://"):
            package_name = tree.path[len("package://") :].split("/", 1)[0]
            package_path = ament_index_python.get_package_share_directory(
                package_name=package_name
            )
            file_path = (
                package_path + tree.path[len("package://") + len(package_name) :]
            )

        else:
            response.success = False
            response.error_message = (
                f'Tree path "{tree.path}" is malformed. It needs to start with '
                f'either "file://" or "package://"'
            )
            return response

        # load tree file and parse yaml, then convert to Tree message
        try:
            tree_file = open(file_path, "r")
        except IOError as ex:
            response.success = False
            response.error_message = f"Error opening file {file_path}: {str(ex)}"
            return response

        with tree_file:
            tree_yaml = tree_file.read()
            try:
                response = parse_tree_yaml(tree_yaml=tree_yaml)
            except yaml.scanner.ScannerError as ex:
                response.success = False
                response.error_message = (
                    f"Encountered a ScannerError while parsing the tree yaml: {str(ex)}"
                )
                return response
            tree = response.tree
            tree.path = request.tree.path

    response.success = True
    response.tree = tree
    return response


@typechecked
def get_available_nodes(
    request: GetAvailableNodes.Request, response: GetAvailableNodes.Response
) -> GetAvailableNodes.Response:
    """
    List the types of nodes that are currently known.

    This includes all nodes from modules that were passed to our
    constructor in `module_list`, ones from modules that nodes have
    been successfully loaded from since launch, and ones from
    modules explicitly asked for in `request.node_modules`

    :param ros_bt_py_msgs.srv.GetAvailableNodesRequest request:

    If `request.node_modules` is not empty, try to load those
    modules before responding.

    :returns: :class:`ros_bt_py_msgs.src.GetAvailableNodesResponse`
    """
    for module_name in request.node_modules:
        if module_name and load_node_module(module_name) is None:
            response.success = False
            response.error_message = f"Failed to import module {module_name}"
            return response

    @typechecked
    def to_node_io(data_map: Dict[str, type | OptionRef]) -> List[NodeIO]:
        return [
            NodeIO(key=name, serialized_type=json_encode(type_or_ref))
            for (name, type_or_ref) in data_map.items()
        ]

    def to_node_option(data_map):
        return [
            NodeOption(key=name, serialized_type=json_encode(type_or_ref))
            for (name, type_or_ref) in data_map.items()
        ]

    response.available_nodes = []
    for module, nodes in Node.node_classes.items():
        for class_name, node_classes in nodes.items():
            for node_class in node_classes:
                if not node_class._node_config:
                    rclpy.logging.get_logger("get_available_nodes").warn(
                        f"Node class: {node_class.__name__} does not have node config!"
                    )
                    continue
                max_children = node_class._node_config.max_children
                max_children = -1 if max_children is None else max_children
                doc = inspect.getdoc(node_class) or ""
                response.available_nodes.append(
                    DocumentedNode(
                        module=module,
                        node_class=class_name,
                        version=node_class._node_config.version,
                        max_children=max_children,
                        options=to_node_option(node_class._node_config.options),
                        inputs=to_node_io(node_class._node_config.inputs),
                        outputs=to_node_io(node_class._node_config.outputs),
                        doc=str(doc),
                        tags=node_class._node_config.tags,
                    )
                )

    response.success = True
    return response


class TreeManager:
    """
    Provide methods to manage a Behavior Tree.

    These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(
        self,
        ros_node: rclpy.node.Node,
        tree_id: Optional[uuid.UUID] = None,
        name: Optional[str] = None,
        module_list: Optional[List[str]] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        tick_frequency_hz: float = 10.0,
        publish_tree_structure_callback: Optional[
            Callable[[TreeStructureList], None]
        ] = None,
        publish_tree_state_callback: Optional[Callable[[TreeStateList], None]] = None,
        publish_tree_data_callback: Optional[Callable[[TreeDataList], None]] = None,
        publish_diagnostic_callback: Optional[Callable[[DiagnosticArray], None]] = None,
        publish_tick_frequency_callback: Optional[Callable[[Float64], None]] = None,
        diagnostics_frequency: float = 1.0,
    ) -> None:
        self.ros_node = ros_node
        if name is None:
            self.name = "UNKNOWN TREE"
        else:
            self.name = name

        if tree_id is None:
            self.tree_id = uuid.UUID(int=0)
        else:
            self.tree_id = tree_id

        self.publish_tree_structure = publish_tree_structure_callback
        if self.publish_tree_structure is None:
            get_logger("tree_manager").get_child(name).info(
                "No callback for publishing tree structure provided."
            )
        self.publish_tree_state = publish_tree_state_callback
        if self.publish_tree_state is None:
            get_logger("tree_manager").get_child(name).info(
                "No callback for publishing tree state provided."
            )
        self.publish_tree_data = publish_tree_data_callback
        if self.publish_tree_data is None:
            get_logger("tree_manager").get_child(name).info(
                "No callback for publishing tree data provided."
            )

        self.publish_diagnostic = publish_diagnostic_callback
        if self.publish_diagnostic is None:
            get_logger("tree_manager").get_child(name).info(
                "No callback for publishing node diagnostics provided"
            )

        self.publish_tick_frequency = publish_tick_frequency_callback
        if self.publish_tick_frequency is None:
            get_logger("tree_manager").get_child(name).info(
                "No callback for publishing tree frequency provided"
            )

        self.debug_manager: DebugManager
        if debug_manager is None:
            get_logger("tree_manager").get_child(name).info(
                "Tree manager instantiated without explicit debug manager "
                "- building our own with default parameters"
            )
            self.debug_manager = DebugManager(ros_node=self.ros_node)
        else:
            self.debug_manager = debug_manager

        self.subtree_manager: SubtreeManager
        if subtree_manager is None:
            get_logger("tree_manager").get_child(name).info(
                "Tree manager instantiated without explicit subtree manager "
                "- building our own with default parameters"
            )
            self.subtree_manager = SubtreeManager()
        else:
            self.subtree_manager = subtree_manager

        if tick_frequency_hz == 0.0:
            tick_frequency_hz = 10.0

        self.tick_sliding_window = [tick_frequency_hz] * 10

        self.nodes: Dict[uuid.UUID, Node] = {}

        self._tree_lock = Lock()
        self._edit_lock = RLock()
        # Stop the tick thread after a single tick
        self._once: bool = False
        # Stop the tick thread after the tree returns something other than
        # RUNNING for the first time
        self._stop_after_result: bool = False

        self.tree_structure = TreeStructure(tree_id=uuid_to_ros(self.tree_id))
        # These reassignments makes the typing happy,
        #   because they ensure that `.append .extent .remove ...` exists
        self.tree_structure.data_wirings = []
        self.tree_structure.public_node_data = []
        self.tree_structure.name = self.name
        self.tree_structure.tick_frequency_hz = tick_frequency_hz
        self.rate = self.ros_node.create_rate(self.tree_structure.tick_frequency_hz)

        self.tree_state = TreeState(tree_id=uuid_to_ros(self.tree_id))

        self.tree_data = TreeData(tree_id=uuid_to_ros(self.tree_id))
        self.enable_publish_data = False

        self.diagnostic_array = DiagnosticArray()
        self.diagnostic_status = DiagnosticStatus()
        self.diagnostic_array.status = [self.diagnostic_status]

        self._last_error: Optional[str] = None

        self.state = TreeState.EDITABLE
        self._tick_thread: Optional[Thread] = None

        # Skip if module_list is empty or None
        if module_list:
            for module_name in module_list:
                load_node_module(module_name)
        self.publish_structure()
        # Also publish data to overwrite any stale messages (relevant after restart)
        #   state is published implicitly
        self.publish_data()

        if self.publish_diagnostic is not None:
            self.ros_node.create_timer(
                1.0 / diagnostics_frequency, self.diagnostic_callback
            )

    @property
    def state(self) -> str:
        return self.tree_state.state

    @state.setter
    @typechecked
    def state(self, new_state: str) -> None:
        with self._tree_lock:
            self.tree_state.state = new_state
            self.ros_node.get_logger().debug(
                f"Updating tree state to {self.tree_state.state}"
            )

    def set_diagnostics_name(self) -> None:
        """
        Set the tree name for ROS diagnostics.

        If the BT has a name, this name will published in diagnostics.
        Otherwise, the root name of the tree is used.
        """
        if self.tree_structure.name:
            self.diagnostic_status.name = os.path.splitext(self.tree_structure.name)[0]
        elif self.tree_structure.root_id:
            self.diagnostic_status.name = self.nodes[
                ros_to_uuid(self.tree_structure.root_id).unwrap()
            ].name
            get_logger("tree_manager").get_child(self.name).warn(
                "No tree name was found. Diagnostics data from the behavior tree will be"
                f"published under the name of the root_node: {self.diagnostic_status.name}"
            )
        else:
            self.diagnostic_status.name = ""
            get_logger("tree_manager").get_child(self.name).warn(
                "Neither a tree name nor the name from the root_node was found."
                "Diagnostics data from the behavior tree will be "
                "published without further name specifications"
            )

    def clear_diagnostics_name(self) -> None:
        """Clear the name for ROS diagnostics."""
        self.diagnostic_status.name = ""

    def diagnostic_callback(self) -> None:
        if self.publish_diagnostic is None:
            return
        if self.state == TreeState.TICKING:
            self.diagnostic_status.level = DiagnosticStatus.OK
            self.diagnostic_status.message = "Ticking"
            # self.tick_stat.values = [KeyValue(key = 'Ticking', value = 'True')]
        elif self.state in (
            TreeState.EDITABLE,
            TreeState.IDLE,
            TreeState.WAITING_FOR_TICK,
            TreeState.STOP_REQUESTED,
        ):
            self.diagnostic_status.level = DiagnosticStatus.WARN
            self.diagnostic_status.message = "Not ticking"
            # self.tick_stat.values = [KeyValue(key = 'Ticking', value = 'False')]
        elif self.state == TreeState.ERROR:
            self.diagnostic_status.level = DiagnosticStatus.ERROR
            self.diagnostic_status.message = "Error in Behavior Tree"
        self.publish_diagnostic(self.diagnostic_array)

    def publish_structure(
        self,
    ):
        """
        Publish the current tree structure using the callback supplied to the constructor.

        This also triggers a state publish.

        In most cases, you'll want that callback to publish to a ROS
        topic.
        """
        if self.publish_tree_structure:
            structure_list = TreeStructureList()
            structure_list.tree_structures = (
                self.subtree_manager.get_subtree_structures()
            )
            structure_list.tree_structures.append(self.structure_to_msg())
            self.publish_tree_structure(structure_list)
        self.publish_state()

    def publish_state(
        self,
    ):
        """
        Publish the current tree state using the callback supplied to the constructor.

        In most cases, you'll want that callback to publish to a ROS
        topic.
        """
        if self.publish_tree_state:
            state_list = TreeStateList()
            state_list.tree_states = self.subtree_manager.get_subtree_states()
            state_list.tree_states.append(self.state_to_msg())
            self.publish_tree_state(state_list)

    def publish_data(
        self,
    ):
        """
        Publish the current tree data using the callback supplied to the constructor.

        This also checks if data publishing is enabled, so it's safe to call either way.
        It will always trigger a state publish either way.

        In most cases, you'll want that callback to publish to a ROS
        topic.
        """
        if self.publish_tree_data and self.enable_publish_data:
            data_list = TreeDataList()
            data_list.tree_data = self.subtree_manager.get_subtree_data()
            data_list.tree_data.append(self.data_to_msg())
            self.publish_tree_data(data_list)
        self.publish_state()

    @typechecked
    def find_root(self) -> Result[Optional[Node], TreeTopologyError]:
        """
        Find the root node of the tree.

        :raises: `TreeTopologyError`

        if nodes exist, but either no root or multiple roots are
        found.
        """
        # TODO This case being an Ok causes a lot of extra checks down the line
        #   Maybe reevaluate if that makes sense.
        if not self.nodes:
            return Ok(None)
        # Find root node
        possible_roots = [node for node in self.nodes.values() if not node.parent]

        if len(possible_roots) > 1:
            return Err(
                TreeTopologyError(
                    f'Tree "{self.tree_structure.name}" has multiple nodes without parents.'
                )
            )
        if not possible_roots:
            return Err(
                TreeTopologyError(
                    f'All nodes in tree "{self.tree_structure.name} have parents. You have '
                    "made a cycle, which makes the tree impossible to run!"
                )
            )
        return Ok(possible_roots[0])

    def tick_report_exceptions(self) -> None:
        """Wrap :meth:`TreeManager.tick()` and catch *all* errors."""
        tick_result = self.tick()

        if tick_result.is_err():
            get_logger("tree_manager").get_child(self.name).error(
                f"Encountered error while ticking tree: {tick_result.unwrap_err()}"
            )
            self._last_error = f"{tick_result.unwrap_err()}"
            self.state = TreeState.ERROR
            self.publish_state()

    @typechecked
    def tick(
        self,
    ) -> Result[None, MissingParentError | BehaviorTreeException | TreeTopologyError]:
        """
        Execute a tick, starting from the tree's root.

        This behaves differently based on the current configuration of
        the `TreeManager` - it can tick once, continuously, until the
        tree reports a result (either SUCCEEDED or FAILED).

        This method should *NOT* be called directly, but rather
        triggered via :meth:`TreeManager.control_execution()`!
        """
        # First check for nodes with missing parents
        orphans = [
            f'"{node.name}"(parent: {node.parent.name if node.parent else ""}")'
            for node in self.nodes.values()
            if node.parent and node.parent.node_id not in self.nodes
        ]
        if orphans:
            return Err(
                MissingParentError(
                    f'The following nodes\' parents are missing: {", ".join(orphans)}'
                )
            )

        root_result = self.find_root()
        if root_result.is_err():
            get_logger(self.name).error("Could not find tree root!")
            return Err(root_result.unwrap_err())

        root = root_result.unwrap()
        if not root:
            get_logger("tree_manager").info(
                "No nodes in tree, tick will not do anything"
            )
            return Err(
                TreeTopologyError("No nodes in  the tree, tick will do nothing!")
            )

        with self._tree_lock:
            self.tree_structure.root_id = uuid_to_ros(root.node_id)
        if root.state in (BTNodeState.UNINITIALIZED, BTNodeState.SHUTDOWN):
            root.setup()
            if root.state is not BTNodeState.IDLE:
                self.state = TreeState.ERROR
                self.publish_state()
                return Err(BehaviorTreeException("Tree not in idle state after setup!"))

        while True:
            tick_start_timestamp = self.ros_node.get_clock().now()
            if self.state == TreeState.STOP_REQUESTED:
                break

            tick_result = root.tick()

            if tick_result.is_err():
                tick_err = tick_result.unwrap_err()
                get_logger(self.name).error(f"Ticking the tree failed: {tick_err}")
                return Err(tick_err)

            self.publish_data()

            tick_result_state = tick_result.unwrap()

            if self._stop_after_result:
                if tick_result_state in [BTNodeState.FAILED, BTNodeState.SUCCEEDED]:
                    break

            if self._once:
                # Return immediately, not unticking anything
                self._once = False
                self.state = TreeState.WAITING_FOR_TICK
                self.publish_state()
                return Ok(None)

            tick_end_timestamp = self.ros_node.get_clock().now()

            duration: Duration = tick_end_timestamp - tick_start_timestamp  # type: ignore
            # We know that Time - Time = Duration
            tick_rate = self.tree_structure.tick_frequency_hz

            if (1 / tick_rate) > (duration.nanoseconds * 1e9):
                get_logger("tree_manager").get_child(self.name).warn(
                    "Tick took longer than set period, cannot tick at "
                    f"{self.tree_structure.tick_frequency_hz:.2f} Hz"
                )

            self.tick_sliding_window.pop(0)
            self.tick_sliding_window.append(duration.nanoseconds * 1e9)
            tick_frequency_avg = sum(self.tick_sliding_window) / len(
                self.tick_sliding_window
            )

            if self.publish_tick_frequency is not None:
                tick_frequency_msg = Float64()
                tick_frequency_msg.data = tick_frequency_avg
                self.publish_tick_frequency(tick_frequency_msg)
            self.rate.sleep()

        self.state = TreeState.IDLE
        self.publish_state()

        # Ensure all nodes are stopped and not doing anything in
        # the background.
        untick_result = root.untick()
        return untick_result.map(lambda x: None)

    @typechecked
    def find_nodes_in_cycles(self) -> List[uuid.UUID]:
        """Return a list of all nodes in the tree that are part of cycles."""
        safe_node_ids: List[uuid.UUID] = []
        nodes_in_cycles: List[uuid.UUID] = []
        # Follow the chain of parent nodes for each node name in self.nodes
        for starting_id in self.nodes.keys():
            cycle_candidates = [starting_id]
            current_node = self.nodes[starting_id]
            while current_node.parent:
                current_node = self.nodes[current_node.parent.node_id]
                cycle_candidates.append(current_node.node_id)
                if current_node.node_id == starting_id:
                    nodes_in_cycles.extend(cycle_candidates)
                    break
                if current_node.node_id in safe_node_ids:
                    # We've already checked for cycles from the parent node, no
                    # need to do that again.
                    safe_node_ids.extend(cycle_candidates)
                    break
            if not current_node.parent:
                safe_node_ids.extend(cycle_candidates)

        return nodes_in_cycles

    ####################
    # Service Handlers #
    ####################

    @is_edit_service
    @typechecked
    def clear(
        self, request: Optional[ClearTree.Request], response: ClearTree.Response
    ) -> ClearTree.Response:
        response.success = False
        root_result = self.find_root()
        if root_result.is_err():
            get_logger("tree_manager").get_child(self.name).warn(
                f"Could not find root {root_result.unwrap_err()}"
            )
            response.error_message = str(root_result.unwrap_err())
            response.success = False
            return response
        root = root_result.unwrap()
        if not root:
            # No root, no problems
            response.success = True
            return response
        if not (root.state in [BTNodeState.UNINITIALIZED, BTNodeState.SHUTDOWN]):
            get_logger("tree_manager").get_child(self.name).error(
                "Please shut down the tree before clearing it"
            )
            response.success = False
            response.error_message = "Please shut down the tree before clearing it"
            return response

        self.nodes = {}
        with self._tree_lock:
            self.tree_structure = TreeStructure(
                tree_id=uuid_to_ros(self.tree_id),
                name="",
                tick_frequency_hz=self.tree_structure.tick_frequency_hz,
            )
            # These reassignments makes the typing happy,
            #   because they ensure that `.append .extent .remove ...` exists
            self.tree_structure.data_wirings = []
            self.tree_structure.public_node_data = []
            self.tree_state = TreeState(
                tree_id=uuid_to_ros(self.tree_id), state=TreeState.EDITABLE
            )
            self.tree_data = TreeData(tree_id=uuid_to_ros(self.tree_id))
        self.publish_structure()
        self.subtree_manager.clear_subtrees()
        self.clear_diagnostics_name()
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def load_tree_from_path(
        self, request: LoadTreeFromPath.Request, response: LoadTreeFromPath.Response
    ) -> LoadTreeFromPath.Response:
        """Wrap around load_tree for convenience."""
        tree = TreeStructure()
        tree.path = request.path
        load_tree_request = LoadTree.Request(tree=tree, permissive=request.permissive)
        load_tree_response = LoadTree.Response()
        load_tree_response = self.load_tree(
            request=load_tree_request, response=load_tree_response
        )

        response.success = load_tree_response.success
        response.error_message = load_tree_response.error_message

        return response

    @is_edit_service
    @typechecked
    def load_tree(  # noqa: C901
        self, request: LoadTree.Request, response: LoadTree.Response
    ) -> LoadTree.Response:
        """
        Load a tree from the given message (which may point to a file).

        :param ros_bt_py_msgs.srv.LoadTree request:

        `request.tree` describes the tree to be loaded, including
        nodes, wirings and public node data.

        If the `Tree` message itself isn't populated, but contains a
        `path` to load a tree from, we open the file it points to and
        load that.
        """
        migrate_tree_request = MigrateTree.Request()
        migrate_tree_request.tree = request.tree
        load_response = MigrateTree.Response()
        load_response = load_tree_from_file(
            request=migrate_tree_request, response=load_response
        )
        if not load_response.success:
            response.error_message = load_response.error_message
            return response

        tree = load_response.tree

        tree.tree_id = uuid_to_ros(self.tree_id)

        for public_datum in tree.public_node_data:
            if public_datum.data_kind == NodeDataLocation.OPTION_DATA:
                response.success = False
                response.error_message = (
                    "public_node_data: option values cannot be public!"
                )

                return response

        try:
            # Clear existing tree, then replace it with the message's contents
            self.clear(None, ClearTree.Response())
            # First just add all nodes to the tree, then restore tree structure
            for node in tree.nodes:
                match self.instantiate_node_from_msg(
                    node_msg=node,
                    ros_node=self.ros_node,
                    permissive=request.permissive,
                ):
                    case Err(e):
                        response.success = False
                        response.error_message = str(e)
                        return response
                    case Ok(node):
                        self.nodes[node.node_id] = node

            for node in tree.nodes:
                # We just parsed all ids before, so we know them to be safe
                node_id = ros_to_uuid(node.node_id).unwrap()
                for c_id in node.child_ids:
                    match ros_to_uuid(c_id):
                        case Err(e):
                            response.success = False
                            response.error_message = e
                            return response
                        case Ok(u):
                            child_id = u
                    match self.nodes[node_id].add_child(self.nodes[child_id]):
                        case Err(e):
                            response.success = False
                            response.error_message = str(e)
                            return response
                        case Ok(_):
                            pass

            # All nodes are added, now do the wiring
            wire_response = WireNodeData.Response()
            wire_response = self.wire_data(
                WireNodeData.Request(wirings=tree.data_wirings, ignore_failure=True),
                response=wire_response,
            )
            if not get_success(wire_response):
                response.success = False
                response.error_message = get_error_message(wire_response)
                return response

            updated_wirings = []
            for wiring in tree.data_wirings:
                if wiring in self.tree_structure.data_wirings:
                    updated_wirings.append(wiring)

            tree.data_wirings = updated_wirings

            self.tree_structure = tree
            # These reassignments makes the typing happy,
            #   because they ensure that `.append .extent .remove ...` exists
            self.tree_structure.data_wirings = list(tree.data_wirings)
            self.tree_structure.public_node_data = list(tree.public_node_data)
            if self.tree_structure.tick_frequency_hz == 0.0:
                get_logger("tree_manager").get_child(self.name).warn(
                    "Tick frequency of loaded tree is 0, defaulting to 10Hz"
                )
                self.tree_structure.tick_frequency_hz = 10.0

            self.rate = self.ros_node.create_rate(
                frequency=self.tree_structure.tick_frequency_hz
            )

            # Ensure Tree is editable after loading
            self.tree_state = TreeState(tree_id=tree.tree_id)
            self.state = TreeState.EDITABLE

            self.tree_data = TreeData(tree_id=tree.tree_id)

            # find and set root name
            find_root_result = self.find_root()
            if find_root_result.is_err():
                response.success = False
                response.error_message = (
                    f"Could not find root of new tree: {find_root_result.unwrap_err()}"
                )
                return response
            root = find_root_result.unwrap()
            if root:
                with self._tree_lock:
                    self.tree_structure.root_id = uuid_to_ros(root.node_id)

            response.success = True
            get_logger("tree_manager").get_child(self.name).info(
                "Successfully loaded tree"
            )
            if self.publish_diagnostic is None:
                self.set_diagnostics_name()
            return response
        finally:
            self.publish_structure()

    @typechecked
    def set_publish_subtrees(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        """
        Set the parameters of our :class:`SubtreeManager`.

        :param  std_srvs.srv.SetBool request:
        """
        if self.subtree_manager:
            self.subtree_manager.publish_subtrees = request.data
            self.publish_structure()
            response.success = True
        else:
            response.success = False
            response.message = "Tree manager has no subtree manager."
        return response

    def set_publish_data(self, request: SetBool.Request, response: SetBool.Response):
        self.enable_publish_data = request.data
        self.subtree_manager.set_publish_data(request.data)

        # Clear data after disabling publish
        if not request.data and self.publish_tree_data:
            self.publish_tree_data(TreeDataList())

        response.success = True
        return response

    @typechecked
    def _control_execution_shutdown(
        self,
        request: ControlTreeExecution.Request,
        response: ControlTreeExecution.Response,
    ) -> ControlTreeExecution.Response:

        find_root_result = self.find_root()
        if find_root_result.is_err():
            response.success = False
            response.error_message = (
                "Failed to determine tree root:" f"{str(find_root_result.unwrap_err())}"
            )
            return response
        root = find_root_result.unwrap()
        if root:
            shutdown_result = root.shutdown()
            if shutdown_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to shutdown: " f"{str(shutdown_result.unwrap_err())}"
                )
                return response
        else:
            get_logger("tree_manager").get_child(self.name).info(
                "Shutting down a tree with no nodes."
            )
        self.state = TreeState.EDITABLE
        response.tree_state = self.state
        response.success = True
        return response

    @typechecked
    def _control_execution_tick_once(
        self,
        request: ControlTreeExecution.Request,
        response: ControlTreeExecution.Response,
    ) -> ControlTreeExecution.Response:
        if self._tick_thread and self._tick_thread.is_alive():
            response.success = False
            response.error_message = (
                "Tried to tick when tree is already running, aborting"
            )
            get_logger("tree_manager").get_child(self.name).warn(response.error_message)
            return response

        else:
            if not self._tick_thread:
                self._tick_thread = Thread(target=self.tick_report_exceptions)
            find_root_result = self.find_root()
            if find_root_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to detrmine tree root: "
                    f"{str(find_root_result.unwrap_err())}"
                )
                response.tree_state = self.state
                return response
            root = find_root_result.unwrap()
            if not root:
                response.success = True
                response.tree_state = self.state
                return response

            self._once = True
            self._stop_after_result = False
            self.state = TreeState.TICKING

            self._tick_thread.start()
            # Give the tick thread some time to finish
            self._tick_thread.join((1.0 / self.tree_structure.tick_frequency_hz) * 4.0)
            # If we're debugging or setting up (and ROS is not
            # shutting down), keep sleepin until the thread
            # finishes
            while self._tick_thread.is_alive() and ok():
                self._tick_thread.join(
                    (1.0 / self.tree_structure.tick_frequency_hz) * 4.0
                )
            if self._tick_thread.is_alive():
                response.success = False
                response.error_message = (
                    "Tried to join tick thread after single tick, but failed!"
                )
                response.tree_state = self.state
                return response

            state_after_joining = self.state
            if state_after_joining == TreeState.WAITING_FOR_TICK:
                response.tree_state = TreeState.WAITING_FOR_TICK
                response.success = True
            elif state_after_joining == TreeState.ERROR:
                response.error_message = (
                    f"Error during single tick: {str(self._last_error)}"
                )
                response.success = False
                get_logger("tree_manager").get_child(self.name).error(
                    response.error_message
                )
            else:
                response.error_message = (
                    f"Successfully stopped ticking, but tree state "
                    f"is {state_after_joining}, not IDLE"
                )
                response.success = False
                get_logger("tree_manager").get_child(self.name).error(
                    response.error_message
                )
            return response

    @typechecked
    def _control_execution_tick_multiple(
        self,
        request: ControlTreeExecution.Request,
        response: ControlTreeExecution.Response,
    ) -> ControlTreeExecution.Response:
        if self._tick_thread and self._tick_thread.is_alive():
            response.success = False
            response.error_message = (
                "Tried to start periodic ticking when tree is "
                "already running, aborting"
            )
            response.tree_state = self.state
            get_logger("tree_manager").get_child(self.name).warn(response.error_message)
            return response

        else:
            if not self._tick_thread:
                self._tick_thread = Thread(target=self.tick_report_exceptions)

            find_root_result = self.find_root()
            if find_root_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to detrmine tree root: "
                    f"{str(find_root_result.unwrap_err())}"
                )
                response.tree_state = self.state
                return response
            root = find_root_result.unwrap()
            if not root:
                response.success = True
                response.tree_state = self.state
                return response
            self.state = TreeState.TICKING
            self._once = False
            self._stop_after_result = False
            if request.command == ControlTreeExecution.Request.TICK_UNTIL_RESULT:
                self._stop_after_result = True
            # Use provided tick frequency, if any
            if request.tick_frequency_hz != 0:
                self.tree_structure.tick_frequency_hz = request.tick_frequency_hz
                self.rate = self.ros_node.create_rate(
                    frequency=self.tree_structure.tick_frequency_hz
                )
            if self.tree_structure.tick_frequency_hz == 0:
                get_logger("tree_manager").get_child(self.name).warn(
                    "Loaded tree had frequency 0Hz. Defaulting to 10Hz"
                )
                self.tree_structure.tick_frequency_hz = 10.0
                self.rate = self.ros_node.create_rate(
                    frequency=self.tree_structure.tick_frequency_hz
                )
            self._tick_thread.start()
            response.success = True
            response.tree_state = TreeState.TICKING
            response.tree_state = self.state
            return response

    @typechecked
    def _control_execution_reset(
        self,
        request: ControlTreeExecution.Request,
        response: ControlTreeExecution.Response,
    ) -> ControlTreeExecution.Response:
        if self._tick_thread and self._tick_thread.is_alive():
            response.success = False
            response.error_message = "Tried to reset tree while it is running, aborting"
            response.tree_state = self.state
            get_logger("tree_manager").get_child(self.name).warn(response.error_message)
            return response
        else:
            find_root_result = self.find_root()
            if find_root_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to determine tree root: "
                    f"{str(find_root_result.unwrap_err())}"
                )
                response.tree_state = self.state
                return response
            root = find_root_result.unwrap()
            if not root:
                get_logger("tree_manager").get_child(self.name).info(
                    "Resetting a tree with no root."
                )
                self.state = TreeState.IDLE
                response.success = True
                response.tree_state = self.state
                return response
            reset_result = root.reset()
            if reset_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to reset tree:" f"{str(reset_result.unwrap_err())}"
                )
                response.tree_state = self.state
                return response

            self.state = TreeState.IDLE
            response.success = True
            response.tree_state = self.state
            return response

    @typechecked
    def control_execution(  # noqa: C901
        self,
        request: ControlTreeExecution.Request,
        response: ControlTreeExecution.Response,
    ) -> ControlTreeExecution.Response:
        """
        Control tree execution.

        :param ros_bt_py_msgs.srv.ControlTreeExecutionRequest request:

        Can request a tick, periodic ticking, periodic ticking until
        the root node reports a result (SUCCEEDED or FAILED), or to
        stop or reset the entire tree.

        """
        response.success = False
        if self._tick_thread is not None:
            is_idle = self.state == TreeState.IDLE
            if is_idle and self._tick_thread.is_alive():
                self._tick_thread.join(0.5)
                if self._tick_thread.is_alive():
                    response.success = False
                    response.error_message = (
                        "Tried to join tick thread with " "Tree state IDLE, but failed!"
                    )
                    self.publish_state()
                    return response

        # Make a new tick thread if there isn't one or the old one has been
        # successfully joined.
        if self._tick_thread is None or not self._tick_thread.is_alive():
            self._tick_thread = Thread(target=self.tick_report_exceptions)

        tree_state = self.state

        # Check for error state and abort if command is not SHUTDOWN -
        # if it is, we fall through to the if below and shut down the
        # tree
        if (
            tree_state == TreeState.ERROR
            and request.command != ControlTreeExecution.Request.SHUTDOWN
        ):
            response.error_message = (
                "Tree is in error state, the only allowed action is SHUTDOWN"
            )
            return response

        if request.command == ControlTreeExecution.Request.SETUP_AND_SHUTDOWN:
            if self._tick_thread.is_alive() or tree_state == TreeState.TICKING:
                response.success = False
                response.error_message = (
                    "Tried to setup tree while it is running, aborting"
                )
                response.tree_state = tree_state
                get_logger("tree_manager").get_child(self.name).warn(
                    response.error_message
                )
                return response

            if self.subtree_manager:
                self.subtree_manager.clear_subtrees()
            find_root_result = self.find_root()
            if find_root_result.is_err():
                response.success = False
                response.error_message = str(find_root_result.unwrap_err())
                response.tree_state = self.state
                self.publish_state()
                return response
            root = find_root_result.unwrap()

            response.tree_state = tree_state
            # shutdown the tree after the setup and shutdown request
            request.command = ControlTreeExecution.Request.SHUTDOWN

        if request.command in [
            ControlTreeExecution.Request.STOP,
            ControlTreeExecution.Request.SHUTDOWN,
        ]:
            if tree_state == TreeState.TICKING:

                self.state = TreeState.STOP_REQUESTED
                # Four times the allowed period should be plenty of time to
                # finish the current tick, if the tree has not stopped by then
                # we're in deep trouble.
                if self._tick_thread.is_alive():
                    # Give the tick thread some time to finish
                    self._tick_thread.join(
                        (1.0 / self.tree_structure.tick_frequency_hz) * 4.0
                    )

                    # If we're debugging or setting up (and ROS is not
                    # shutting down), keep sleeping until the thread
                    # finishes
                    while self._tick_thread.is_alive() and ok():
                        self._tick_thread.join(
                            (1.0 / self.tree_structure.tick_frequency_hz) * 4.0
                        )
                    if self._tick_thread.is_alive():
                        response.success = False
                        response.error_message = (
                            "Tried to join tick thread after requesting "
                            "stop, but failed!"
                        )
                        return response

                state_after_joining = self.state
                if state_after_joining == TreeState.IDLE:
                    response.tree_state = TreeState.IDLE
                    response.success = True

                elif state_after_joining == TreeState.ERROR:
                    response.error_message = (
                        f"Error stopping tick: {str(self._last_error)}"
                    )
                    response.success = False
                    get_logger("tree_manager").get_child(self.name).error(
                        response.error_message
                    )
                    return response
                else:
                    response.error_message = (
                        f"Successfully stopped ticking, but tree state is "
                        f"{state_after_joining}, not IDLE"
                    )
                    response.success = False
                    get_logger("tree_manager").get_child(self.name).error(
                        response.error_message
                    )
                    return response

            elif tree_state == TreeState.WAITING_FOR_TICK:
                find_root_result = self.find_root()

                if find_root_result.is_err():
                    response.success = False
                    response.error_message = (
                        "Could not determine tree root: "
                        f"{str(find_root_result.unwrap_err())}"
                    )
                    return response
                root = find_root_result.unwrap()
                if root:
                    root.untick()
                    state = root.state
                    if state in [BTNodeState.IDLE, BTNodeState.PAUSED]:
                        response.tree_state = TreeState.IDLE
                        response.success = True
                    else:
                        response.tree_state = TreeState.ERROR
                        response.success = False
                        get_logger("tree_manager").get_child(self.name).error(
                            f"Root node ({str(root)}) state after unticking is neither "
                            f"IDLE nor PAUSED, but {state}"
                        )
                        response.error_message = "Failed to untick root node."
                        return response
                else:
                    get_logger("tree_manager").get_child(self.name).info(
                        "Unticking a tree with no nodes."
                    )
                    response.tree_state = TreeState.IDLE
                    response.success = True

            else:
                get_logger("tree_manager").get_child(self.name).info(
                    "Received stop command, but tree was not running"
                )

            self.publish_state()

            # actually shut down the tree
            if request.command == ControlTreeExecution.Request.SHUTDOWN:
                response = self._control_execution_shutdown(request, response)
                self.publish_state()

        elif request.command == ControlTreeExecution.Request.TICK_ONCE:
            response = self._control_execution_tick_once(request, response)

        elif request.command in [
            ControlTreeExecution.Request.TICK_PERIODICALLY,
            ControlTreeExecution.Request.TICK_UNTIL_RESULT,
        ]:
            response = self._control_execution_tick_multiple(request, response)

        elif request.command == ControlTreeExecution.Request.RESET:
            response = self._control_execution_reset(request, response)
            self.publish_state()

        elif request.command == ControlTreeExecution.Request.DO_NOTHING:
            get_logger("tree_manager").get_child(self.name).info(
                "Doing nothing in this request"
            )
            response.success = True
        else:
            response.error_message = f"Received unknown command {request.command}"
            get_logger("tree_manager").get_child(self.name).error(
                response.error_message
            )
            response.success = False

        return response

    @is_edit_service
    @typechecked
    def add_node(
        self, request: AddNode.Request, response: AddNode.Response
    ) -> AddNode.Response:
        """
        Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeRequest request:
          A request describing the node to add.
        """
        internal_request = AddNodeAtIndex.Request(
            parent_node_id=request.parent_node_id,
            node=request.node,
            new_child_index=-1,
        )
        internal_response = AddNodeAtIndex.Response()
        internal_response = self.add_node_at_index(
            request=internal_request, response=internal_response
        )

        response.success = internal_response.success
        response.error_message = internal_response.error_message
        return response

    @is_edit_service
    @typechecked
    def add_node_at_index(
        self, request: AddNodeAtIndex.Request, response: AddNodeAtIndex.Response
    ) -> AddNodeAtIndex.Response:
        """
        Add the node in this request to the tree.

        The `node_id` from the request is discarded and a new one is randomly generated.
        The actual is that the node is assigned is included in the response.

        :param ros_bt_py_msgs.srv.AddNodeAtIndexRequest request:
            A request describing the node to add.
        """
        node_msg = request.node
        node_msg.node_id = uuid_to_ros(uuid.uuid4())
        instance_result = self.instantiate_node_from_msg(
            node_msg=request.node,
            ros_node=self.ros_node,
        )
        if instance_result.is_err():
            response.success = False
            response.error_message = str(instance_result.unwrap_err())
            return response
        instance = instance_result.unwrap()

        response.success = True
        response.node_id = node_msg.node_id

        match ros_to_uuid(request.parent_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(p_id):
                parent_node_id = p_id

        # Add node as child of the named parent, if any
        if parent_node_id != uuid.UUID(int=0):
            if parent_node_id not in self.nodes:
                response.success = False
                response.error_message = (
                    f"Parent {request.parent_node_id} of node "
                    f"{instance.name} does not exist!"
                )
                # Remove node from tree
                self.remove_node(
                    request=RemoveNode.Request(
                        node_name=instance.name, remove_children=False
                    ),
                    response=RemoveNode.Response(),
                )
                return response

            add_child_result = self.nodes[parent_node_id].add_child(
                child=instance, at_index=request.new_child_index
            )
            if add_child_result.is_err():
                response.success = False
                response.error_message = str(add_child_result.unwrap_err())
                return response

        # Add children from msg to node
        missing_children = []
        for child_id in request.node.child_ids:
            if child_id in self.nodes:
                add_child_result = instance.add_child(self.nodes[child_id])
                if add_child_result.is_err():
                    get_logger("tree_manager").get_child(self.name).warn(
                        f"Could not add child: {str(add_child_result.unwrap_err())}"
                    )
                    missing_children.append(child_id)
            else:
                missing_children.append(child_id)
        if missing_children:
            response.success = False
            response.error_message = (
                f"Children for node {instance.name} are not or could not be added"
                f"in tree: {str(missing_children)}"
            )
            # Remove node from tree to restore state before insertion attempt
            self.remove_node(
                request=RemoveNode.Request(
                    node_name=instance.name, remove_children=False
                ),
                response=RemoveNode.Response(),
            )

        nodes_in_cycles = self.find_nodes_in_cycles()
        if nodes_in_cycles:
            response.success = False
            response.error_message = (
                f"Found cycles in tree {self.tree_structure.name} after inserting node "
                f"{request.node.name}. Nodes in cycles: {str(nodes_in_cycles)}"
            )
            # First, remove all of the node's children to avoid infinite
            # recursion in remove_node()
            for child_id in [c.node_id for c in instance.children]:
                remove_node_result = instance.remove_child(child_id)
                if remove_node_result.is_err():
                    return response

            # Then remove the node from the tree
            self.remove_node(
                request=RemoveNode.Request(
                    node_name=instance.name, remove_children=False
                ),
                response=RemoveNode.Response(),
            )
            return response
        self.publish_structure()
        return response

    @is_edit_service
    @typechecked
    def reload_tree(
        self, request: Optional[ReloadTree.Request], response: ReloadTree.Response
    ) -> ReloadTree.Response:
        """Reload the currently loaded tree."""
        load_response = LoadTree.Response()
        load_response = self.load_tree(
            request=LoadTree.Request(tree=self.tree_structure), response=load_response
        )

        response.success = load_response.success
        response.error_message = load_response.error_message

        return response

    @is_edit_service
    @typechecked
    def change_tree_name(
        self, request: ChangeTreeName.Request, response: ChangeTreeName.Response
    ) -> ChangeTreeName.Response:
        """Change the name of the currently loaded tree."""
        self.tree_structure.name = request.name
        self.publish_structure()

        response.success = True

        return response

    @is_edit_service
    @typechecked
    def remove_node(
        self, request: RemoveNode.Request, response: RemoveNode.Response
    ) -> RemoveNode.Response:
        """
        Remove the node identified by `request.node_name` from the tree.

        If the parent of the node removed supports enough children to
        take on all of the removed node's children, it will. Otherwise,
        children will be orphaned.
        """
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f"No node with id {request.node_id} in "
                f"tree {self.tree_structure.name}"
            )
            return response

        target_node = self.nodes[node_id]
        # Shutdown node - this should also shutdown all children, but you
        # never know, so check later.
        shutdown_result = target_node.shutdown()
        if shutdown_result.is_err():
            response.success = False
            response.error_message = (
                "Failed to shutdown node to remove: "
                f"{str(shutdown_result.unwrap_err())}"
            )
            return response

        node_ids_to_remove = [target_node.node_id]
        if request.remove_children:
            add_children_of = [target_node.node_id]
            while add_children_of:
                n_id = add_children_of.pop()
                if n_id not in self.nodes:
                    response.success = False
                    response.error_message = (
                        f"Error while removing children of node {target_node.name}: "
                        f"No node with id {n_id} in tree {self.tree_structure.name}"
                    )
                    return response
                node_ids_to_remove.extend(
                    [child.node_id for child in self.nodes[n_id].children]
                )
                add_children_of.extend(
                    [child.node_id for child in self.nodes[n_id].children]
                )

        # Unwire wirings that have removed nodes as source or target
        unwire_response = self.unwire_data(
            WireNodeData.Request(
                wirings=[
                    wiring
                    for wiring in self.tree_structure.data_wirings
                    # Wirings coming from the internal state will have valid node ids
                    if (
                        ros_to_uuid(wiring.source.node_id).unwrap()
                        in node_ids_to_remove
                        or ros_to_uuid(wiring.target.node_id).unwrap()
                        in node_ids_to_remove
                    )
                ]
            ),
            WireNodeData.Response(),
        )
        if not unwire_response.success:
            response.success = False
            response.error_message = (
                "Failed to unwire nodes for removal: "
                f"{unwire_response.error_message}"
            )
            return response

        # Remove nodes in the reverse order they were added to the
        # list, i.e. the "deepest" ones first. This ensures that the
        # parent we refer to in the error message still exists.

        # set to prevent us from removing a node twice - no node
        # *should* have more than one parent, but better safe than
        # sorry
        removed_node_ids = set()
        for n_id in reversed(node_ids_to_remove):
            if n_id in removed_node_ids:
                continue
            removed_node_ids.add(n_id)
            # Check if node is already in shutdown state. If not, call
            # shutdown, but warn, because the parent node should have
            # done that!
            r_node = self.nodes[n_id]
            if r_node.state != BTNodeState.SHUTDOWN:
                # It's reasonable to expect parent to not be None here, since
                # the node is one of a list of children
                if r_node.parent is None:
                    get_logger("tree_manager").get_child(self.name).error(
                        f"Node {r_node.name} appears to be a child with no parent"
                    )
                    continue
                get_logger("tree_manager").get_child(self.name).warn(
                    f"Node {r_node.name} was not shut down. "
                    "Shutdown of child nodes should be handled in the base `Node` class."
                )
                r_node.shutdown()

            # If we have a parent, remove the node from that parent
            # TODO Why this convoluted double lookup, the `parent` reference should be good?
            if (
                r_node.parent is not None
                and r_node.parent.node_id in self.nodes  # type: ignore
            ):
                self.nodes[r_node.parent.node_id].remove_child(r_node.node_id)  # type: ignore
            del self.nodes[r_node.node_id]

        # This is moved down here, because setting the parent to None breaks the unwire
        if not request.remove_children:
            # If we're not removing the children, at least set their parent to None
            for child in target_node.children:
                child.parent = None

        # Keep tree_structure up-to-date
        # TODO The unwire_data call above should already update this
        self.tree_structure.data_wirings = [
            wiring
            for wiring in self.tree_structure.data_wirings
            # Wirings coming from the internal state will have valid node ids
            if (
                ros_to_uuid(wiring.source.node_id).unwrap() not in removed_node_ids
                and ros_to_uuid(wiring.target.node_id).unwrap() not in removed_node_ids
            )
        ]

        self.tree_structure.public_node_data = [
            data
            for data in self.tree_structure.public_node_data
            # Data coming from the internal state will have valid node ids
            if ros_to_uuid(data.node_id).unwrap() not in removed_node_ids
        ]

        for n_id in removed_node_ids:
            self.subtree_manager.remove_subtree(n_id)

        response.success = True
        self.publish_structure()
        return response

    @is_edit_service
    @typechecked
    def morph_node(
        self, request: MorphNode.Request, response: MorphNode.Response
    ) -> MorphNode.Response:
        """Morphs the flow control node into the new node provided in `request.new_node`."""
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f"No node with id {node_id} in" f" tree {self.tree_structure.name}"
            )
            return response

        old_node = self.nodes[node_id]

        new_node_result = Node.from_msg(request.new_node, ros_node=self.ros_node)
        if new_node_result.is_err():
            response.success = False
            response.error_message = (
                "Error instantiating node " f"{str(new_node_result.unwrap_err())}"
            )
            return response
        new_node = new_node_result.unwrap()

        # First unwire all data connection to the existing node
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self.tree_structure.data_wirings
                if old_node.node_id
                in [
                    ros_to_uuid(wiring.source.node_id),
                    ros_to_uuid(wiring.target.node_id),
                ]
            ]
        )

        unwire_resp = WireNodeData.Response()
        unwire_resp = self.unwire_data(request=wire_request, response=unwire_resp)
        if not get_success(unwire_resp):
            return MorphNode.Response(
                success=False,
                error_message=(
                    f"Failed to unwire data for node {old_node.name}: "
                    f"{get_error_message(unwire_resp)}"
                ),
            )

        parent = None
        if old_node.parent:
            parent = old_node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(old_node.node_id)

            if old_child_index is None:
                return MorphNode.Response(
                    success=False,
                    error_message=(
                        f"Parent of node {old_node.name} claims to have no child with that name?!"
                    ),
                )
            remove_child_result = parent.remove_child(old_node.node_id)
            if remove_child_result.is_err():
                response.success = False
                response.error_message = (
                    "Could not remove child from parent: "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response
            add_child_result = parent.add_child(new_node, at_index=old_child_index)
            if add_child_result.is_err():
                response.error_message = (
                    f"Failed to add new instance of node {old_node.name}: "
                    f"{str(add_child_result.unwrap_err())}"
                )
                add_old_node_result = parent.add_child(
                    old_node, at_index=old_child_index
                )
                if add_old_node_result.is_err():
                    response.error_message += "\n Also failed to restore old node."

                rewire_resp = WireNodeData.Response()
                rewire_resp = self.wire_data(request=wire_request, response=rewire_resp)
                if not get_success(rewire_resp):
                    response.error_message += (
                        f"\nAlso failed to restore data wirings: "
                        f"{get_error_message(rewire_resp)}"
                    )
                response.success = False
                return response

        # Move the children from old to new
        for child_id in [child.node_id for child in old_node.children]:
            remove_child_result = old_node.remove_child(child_id)
            if remove_child_result.is_err():
                response.success = False
                response.error_message = (
                    "Could not remove child from old node: "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response
            child = remove_child_result.unwrap()
            if child_id != new_node.node_id:
                add_child_result = new_node.add_child(child)
                if add_child_result.is_err():
                    response.success = False
                    response.error_message = (
                        "Could not add child to new node: "
                        f"{str(add_child_result.unwrap_err())}"
                    )
                    return response

        # Add the new node to self.nodes
        del self.nodes[old_node.node_id]
        self.nodes[new_node.node_id] = new_node

        # Re-wire all the data, just as it was before
        # FIXME: this should be a best-effort rewiring, only re-wire identical input/outputs
        new_wire_request = deepcopy(wire_request)

        rewire_resp = WireNodeData.Response()
        rewire_resp = self.wire_data(request=new_wire_request, response=rewire_resp)
        if not get_success(rewire_resp):
            response.error_message = (
                f"Failed to re-wire data to new node {new_node.name}:"
                f" {get_error_message(rewire_resp)}"
            )
            response.success = False
            return response

        response.success = True
        self.publish_structure()
        return response

    @is_edit_service
    @typechecked
    def set_options(  # noqa: C901
        self, request: SetOptions.Request, response: SetOptions.Response
    ) -> SetOptions.Response:
        """
        Set the option values of a given node.

        This is an "edit service", i.e. it can only be used when the
        tree has not yet been initialized or has been shut down.
        """
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f"Unable to find node {node_id} in tree " f"{self.tree_structure.name}"
            )
            return response

        node = self.nodes[node_id]
        unknown_options = []
        preliminary_incompatible_options = []

        try:
            deserialized_options: Dict[str, Any] = {}
            for option in request.options:
                deserialized_options[option.key] = json_decode(option.serialized_value)
            # deserialized_options = {
            #    (option.key, json_decode(option.serialized_value))
            #    for option in request.options
            # }
        except ValueError as ex:
            response.success = False
            response.error_message = f"Failed to deserialize option value: {str(ex)}"
            return response

        # Find any options values that
        # a) the node does not expect
        # b) have the wrong type
        for key, value in deserialized_options.items():
            if key not in node.options:
                unknown_options.append(key)
                continue

            required_type = node.options.get_type(key)
            if not node.options.compatible(key, value):
                preliminary_incompatible_options.append((key, required_type.__name__))

        error_strings = []
        if unknown_options:
            error_strings.append(f"Unknown option keys: {str(unknown_options)}")

        incompatible_options = []
        if preliminary_incompatible_options:
            # traditionally we would fail here, but re-check if the type of an option
            # with a known option-wiring changed
            # this could mean that the previously incompatible option is actually
            # compatible with the new type!
            for key, required_type_name in preliminary_incompatible_options:
                incompatible = True
                possible_optionref = node.__class__._node_config.options[key]

                if isinstance(possible_optionref, OptionRef):
                    other_type = deserialized_options[possible_optionref.option_key]
                    our_type = type(deserialized_options[key])
                    if other_type == our_type:
                        incompatible = False
                    elif inspect.isclass(other_type):
                        deserialized_options[key] = (
                            message_converter.convert_dictionary_to_ros_message(
                                other_type, deserialized_options[key]
                            )
                        )
                        incompatible = False
                    else:
                        # check if the types are str or unicode and treat them the same
                        if (
                            isinstance(deserialized_options[key], str)
                            and other_type == str
                        ):
                            incompatible = False
                        if (
                            isinstance(deserialized_options[key], str)
                            and other_type == str
                        ):
                            incompatible = False
                if incompatible:
                    incompatible_options.append((key, required_type_name))

        if incompatible_options:
            error_strings.append(
                "Incompatible option keys:\n"
                + "\n".join(
                    [
                        f"Key {key} has type {type(deserialized_options[key]).__name__}, "
                        f"should be {required_type_name}"
                        for key, required_type_name in incompatible_options
                    ]
                )
            )
        if error_strings:
            response.success = False
            response.error_message = "\n".join(error_strings)
            return response

        # Because options are used at construction time, we need to
        # construct a new node with the new options.

        # First, we need to add the option values that didn't change
        # to our dict:
        for key in node.options:
            if key not in deserialized_options:
                deserialized_options[key] = node.options[key]

        # Now we can construct the new node - no need to call setup,
        # since we're guaranteed to be in the edit state
        # (i.e. `root.setup()` will be called before anything that
        # needs the node to be set up)
        try:
            new_node = node.__class__(
                node_id=node_id,
                options=deserialized_options,
                name=request.new_name if request.rename_node else node.name,
                debug_manager=node.debug_manager,
                subtree_manager=node.subtree_manager,
                ros_node=self.ros_node,
            )
        except BehaviorTreeException as exc:
            response.success = False
            response.error_message = str(exc)
            return response

        # Use this request to unwire any data connections the existing
        # node has - if we didn't do this, the node wouldn't ever be
        # garbage collected, among other problems.
        #
        # We'll use the same request to re-wire the connections to the
        # new node (or the old one, if anything goes wrong).
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self.tree_structure.data_wirings
                if node_id
                in [
                    ros_to_uuid(wiring.source.node_id).unwrap(),
                    ros_to_uuid(wiring.target.node_id).unwrap(),
                ]
            ]
        )

        unwire_resp = WireNodeData.Response()
        unwire_resp = self.unwire_data(request=wire_request, response=unwire_resp)
        if not get_success(unwire_resp):
            response.success = False
            response.error_message = (
                f"Failed to unwire data for node {node.name}: "
                f"{get_error_message(unwire_resp)}"
            )
            return response

        parent = None
        if node.parent:
            parent = node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(node.node_id)

            if old_child_index is None:
                response.success = False
                response.error_message = (
                    f"Parent of node {node.name} claims to "
                    f"have no child with that name?!"
                )
                return response

            remove_child_result = parent.remove_child(node.node_id)
            if remove_child_result.is_err():
                error_message = (
                    f"Failed to remove old instance of node {node.name}: "
                    f"{str(remove_child_result.unwrap_err())}"
                )

                rewire_resp = WireNodeData.Response()
                rewire_resp = self.wire_data(request=wire_request, response=rewire_resp)

                if not get_success(rewire_resp):
                    error_message += (
                        "\nAlso failed to restore data wirings: "
                        f"{get_error_message(rewire_resp)}"
                    )

                response.success = False
                response.error_message = error_message
                return response

            add_child_result = parent.add_child(new_node, at_index=old_child_index)
            if add_child_result.is_err():
                error_message = (
                    f"Failed to add new instance of node {node.name}: "
                    f"{str(add_child_result.unwrap_err())}"
                )
                add_child_result = parent.add_child(node, at_index=old_child_index)
                if add_child_result.is_err():
                    error_message += "\n Also failed to restore old node."
                else:

                    rewire_resp = WireNodeData.Response()
                    rewire_resp = self.wire_data(
                        request=wire_request, response=rewire_resp
                    )
                    if not get_success(rewire_resp):
                        error_message += (
                            f"\nAlso failed to restore data wirings: "
                            f"{get_error_message(rewire_resp)}"
                        )
                response.success = False
                response.error_message = error_message
                return response

        # Add the new node to self.nodes
        self.nodes[node_id] = new_node

        rewire_resp = WireNodeData.Response()
        rewire_resp = self.wire_data(request=wire_request, response=rewire_resp)
        if not get_success(rewire_resp):
            error_message = (
                f"Failed to re-wire data to new node {new_node.name}: "
                f"{get_error_message(rewire_resp)}"
            )
            # Try to undo everything, starting with adding the old
            # node back to the node dict
            self.nodes[node_id] = node

            if parent is not None:
                remove_child_result = parent.remove_child(new_node.node_id)
                if remove_child_result.is_err():
                    error_message += (
                        "\nError restoring old node: "
                        f"{str(remove_child_result.unwrap_err())}"
                    )
                else:
                    add_child_result = parent.add_child(node, at_index=old_child_index)
                    if add_child_result.is_err():
                        error_message += (
                            "\nError restoring old node: "
                            f"{str(add_child_result.unwrap_err())}"
                        )

            # Now try to re-do the wirings
            recovery_wire_response = WireNodeData.Response()
            recovery_wire_response = self.wire_data(
                request=wire_request, response=recovery_wire_response
            )
            if not get_success(recovery_wire_response):
                error_message += (
                    f"\nFailed to re-wire data to restored node {node.name}: "
                    f"{get_error_message(recovery_wire_response)}"
                )
            response.success = False
            response.error_message = error_message
            return response

        # This line is important: The list comprehension creates a
        # new list that won't be affected by calling
        # remove_child()!
        for child_id in [child.node_id for child in node.children]:
            get_logger("tree_manager").get_child(self.name).info(
                f"Moving child {child_id}"
            )
            remove_child_result = node.remove_child(child_id)
            if remove_child_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to transfer children to new node: "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response
            child = remove_child_result.unwrap()
            add_child_result = new_node.add_child(child)
            if add_child_result.is_err():
                response.success = False
                response.error_message = (
                    "Failed to transfer children to new node: "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response

        # We made it!
        self.publish_structure()
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def move_node(
        self, request: MoveNode.Request, response: MoveNode.Response
    ) -> MoveNode.Response:
        """Move the named node to a different parent and insert it at the given index."""
        match ros_to_uuid(request.node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                node_id = n_id

        match ros_to_uuid(request.parent_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(p_id):
                parent_node_id = p_id

        if node_id not in self.nodes:
            response.success = False
            response.error_message = f'Node to be moved ("{node_id}") is not in tree.'
            return response

        node = self.nodes[node_id]

        # Empty parent name -> just remove node from parent
        if parent_node_id == uuid.UUID(int=0):
            if node.parent is not None:
                remove_child_result = node.parent.remove_child(node.node_id)
                if remove_child_result.is_err():
                    response.success = False
                    response.error_message = (
                        f"Could not remove child {node.name} from {node.parent.name}: "
                        f"{str(remove_child_result.unwrap_err())}"
                    )
                    return response

            self.publish_structure()
            response.success = True
            return response

        if parent_node_id not in self.nodes:
            response.success = False
            response.error_message = f'New parent ("{parent_node_id}") is not in tree.'
            return response

        new_parent = self.nodes[parent_node_id]

        if (
            new_parent.node_config.max_children is not None
            and len(new_parent.children) == new_parent.node_config.max_children
        ):
            response.success = False
            response.error_message = (
                f"Cannot move node {node.name} to new parent node "
                f"{new_parent.name}. "
                "Parent node already has the maximum number "
                f"of children ({new_parent.node_config.max_children})."
            )
            return response

        # If the new parent is part of the moved node's subtree, we'd
        # get a cycle, so check for that and fail if true!
        get_subtree_msg_result = node.get_subtree_msg()
        if get_subtree_msg_result.is_err():
            response.success = False
            response.error_message = (
                f"Could not generate subtree msg from {node.name}: "
                f"{str(get_subtree_msg_result.unwrap_err())}"
            )
            return response
        subtree_msg = get_subtree_msg_result.unwrap()
        if new_parent.node_id in [
            subtree_node.name for subtree_node in subtree_msg[0].nodes
        ]:
            response.success = False
            response.error_message = (
                f"Cannot move node {node.name} to new parent node "
                f"{new_parent.name}. "
                f"{new_parent.name} is a child of {node.name}!"
            )
            return response

        # Remove node from old parent, if any
        old_parent = node.parent
        if old_parent is not None:
            remove_child_result = old_parent.remove_child(node.node_id)
            if remove_child_result.is_err():
                response.success = False
                response.error_message = (
                    f"Failed to remove child {node.node_id} from {old_parent.name}: "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response

        # Add node to new parent
        add_child_result = new_parent.add_child(
            child=node, at_index=request.new_child_index
        )
        if add_child_result.is_err():
            response.success = False
            response.error_message = (
                f"Failed to add child {node.name} to {new_parent.name}: "
                f"{str(add_child_result.unwrap_err())}"
            )
            return response

        self.publish_structure()
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def replace_node(
        self, request: ReplaceNode.Request, response: ReplaceNode.Response
    ) -> ReplaceNode.Response:
        """
        Replace the named node with `new_node`.

        Will also move all children of the old node to the new one, but
        only if `new_node` supports that number of children. Otherwise,
        this will return an error and leave the tree unchanged.
        """
        match ros_to_uuid(request.old_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                old_node_id = n_id

        match ros_to_uuid(request.new_node_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                new_node_id = n_id

        if old_node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f'Node to be replaced ("{old_node_id}")' "is not in tree."
            )
            return response
        if new_node_id not in self.nodes:
            response.success = False
            response.error_message = (
                f'Replacement node ("{new_node_id}") is not in tree.'
            )
            return response

        old_node = self.nodes[old_node_id]
        new_node = self.nodes[new_node_id]

        if (
            new_node.node_config.max_children is not None
            and len(old_node.children) > new_node.node_config.max_children
        ):
            response.success = False
            response.error_message = (
                f'Replacement node ("{new_node.name}") does not support the number of'
                f"children required ({old_node.name} has "
                f"{len(old_node.children)} children, "
                f"{new_node.name} supports {new_node.node_config.max_children}."
            )
            return response

        # TODO(nberg): Actually implement this

        # If the new node has inputs/outputs with the same name and
        # type as the old one,wire them the same way the old node was
        # wired

        # Note the old node's position in its parent's children array
        old_node_parent = old_node.parent
        # Initialize to 0 just to be sure. We *should* be guaranteed
        # to find the old node in its parent's children array, but
        # better safe than sorry.
        old_node_child_index = 0
        if old_node_parent is not None:
            for index, child in enumerate(old_node_parent.children):
                if child.node_id == old_node.node_id:
                    old_node_child_index = index
                    break

        # If it has the same parent as the old node, check its index, too.
        #
        # If the new node's index is smaller than the old one's, we
        # need to subtract one from old_node_child_index. Imagine we
        # want to replace B with A, and both are children of the same
        # node:
        #
        # parent.children = [A, B, C]
        #
        # Then old_node_child_index would be 1. But if we remove B
        #
        # parent.children = [A, C]
        #
        # And then move A to old_node_child_index, we end up with
        #
        # parent.children = [C, A]
        #
        # Which is wrong!
        if (
            new_node.parent is not None
            and old_node_parent is not None
            and new_node.parent.node_id == old_node_parent.node_id
            and old_node_child_index > 0
        ):
            for index, child in enumerate(new_node.parent.children):
                if child.node_id == new_node.node_id:
                    if index < old_node_child_index:
                        old_node_child_index -= 1
                    break

        # Move the children from old to new
        for child_id in [child.node_id for child in old_node.children]:
            remove_child_result = old_node.remove_child(child_id)
            if remove_child_result.is_err():
                response.success = False
                response.error_message = (
                    f"Could not remove child node: {child_id} "
                    f"{str(remove_child_result.unwrap_err())}"
                )
                return response

            child = remove_child_result.unwrap()
            if child_id != new_node.node_id:
                add_child_result = new_node.add_child(child)
                if add_child_result.is_err():
                    response.success = False
                    response.error_message = (
                        f"Failed to add child {child.name}: "
                        f"{str(add_child_result.unwrap_err())}"
                    )
                    return response

        # Remove the old node (we just moved the children, so we can
        # set remove_children to True)
        res = RemoveNode.Response()
        res = self.remove_node(
            RemoveNode.Request(
                node_id=uuid_to_ros(old_node.node_id), remove_children=True
            ),
            res,
        )

        if not get_success(res):
            # self.publish_structure()
            response.success = False
            response.error_message = (
                f'Could not remove old node: "{get_error_message(res)}"'
            )
            return response

        # Move the new node to the old node's parent (if it had one)
        if old_node_parent is not None:
            move_response = self.move_node(
                MoveNode.Request(
                    node_id=uuid_to_ros(new_node.node_id),
                    parent_node_id=uuid_to_ros(old_node_parent.node_id),
                    new_child_index=old_node_child_index,
                ),
                MoveNode.Response(),
            )
            if not move_response.success:
                response.success = move_response.success
                response.error_message = move_response.error_message
                return response

        self.publish_structure()
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def wire_data(
        self, request: WireNodeData.Request, response: WireNodeData.Response
    ) -> WireNodeData.Response:
        """
        Connect the given pairs of node data to one another.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class: `ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
        """
        response.success = True
        find_root_result = self.find_root()
        if find_root_result.is_err():
            response.success = False
            response.error_message = (
                "Unable to find root node: " f"{str(find_root_result.unwrap_err())}"
            )
            return response
        root = find_root_result.unwrap()
        if not root:
            response.success = False
            response.error_message = "Tree is empty cannot wire!"
            return response

        successful_wirings = []
        for wiring in request.wirings:
            match ros_to_uuid(wiring.target.node_id):
                case Err(e):
                    response.success = False
                    response.error_message = e
                    break
                case Ok(n_id):
                    target_node_id = n_id

            target_node = root.find_node(target_node_id)
            if not target_node:
                response.success = False
                response.error_message = f"Target node {target_node_id} does not exist"
                break
            wire_data_result = target_node.wire_data(wiring)
            if wire_data_result.is_err():
                if not request.ignore_failure:
                    response.success = False
                    response.error_message = (
                        f'Failed to execute wiring "{wiring}": '
                        f"{str(wire_data_result.unwrap_err())}"
                    )
                    break
            else:
                successful_wirings.append(wiring)

        if not response.success:
            # Undo the successful wirings
            for wiring in successful_wirings:
                match ros_to_uuid(wiring.target.node_id):
                    case Err(e):
                        response.success = False
                        response.error_message = e
                        return response
                    case Ok(n_id):
                        target_node_id = n_id

                target_node = root.find_node(target_node_id)
                if not target_node:
                    response.success = False
                    response.error_message = (
                        "Failed to find node target: " f"{target_node_id}"
                    )
                    return response
                unwire_result = target_node.unwire_data(wiring)
                if unwire_result.is_err():
                    response.success = False
                    response.error_message = (
                        f'Failed to undo wiring "{wiring}": {str(unwire_result.unwrap_err())}\n'
                        f"Previous error: {response.error_message}"
                    )
                    get_logger("tree_manager").get_child(self.name).error(
                        "Failed to undo successful wiring after error. "
                        "Tree is in undefined state!"
                    )
                    return response
            return response
        else:
            # only actually wire any data if there were no errors
            # We made it here, so all the Wirings should be valid. Time to save
            # them.
            cast(list, self.tree_structure.data_wirings).extend(successful_wirings)
            self.publish_structure()
        return response

    @is_edit_service
    @typechecked
    def unwire_data(
        self, request: WireNodeData.Request, response: WireNodeData.Response
    ) -> WireNodeData.Response:
        """
        Disconnect the given pairs of node data.

        :param ros_bt_py_msgs.srv.WireNodeDataRequest request:

        Contains a list of :class:`ros_bt_py_msgs.msg.NodeDataWiring`
        objects that model connections

        :returns: :class:`ros_bt_py_msgs.src.WireNodeDataResponse` or `None`
        """
        root_result = self.find_root()
        if root_result.is_err():
            response.success = False
            response.error_message = (
                "Unable to find root node: " f"{str(root_result.unwrap_err())}"
            )
            return response

        root = root_result.unwrap()
        if not root:
            response.success = False
            response.error_message = "Tree is empty cannot unwire data!"
            return response

        response.success = True
        successful_unwirings = []
        for wiring in request.wirings:
            match ros_to_uuid(wiring.target.node_id):
                case Err(e):
                    response.success = False
                    response.error_message = e
                    break
                case Ok(n_id):
                    target_node_id = n_id

            target_node = root.find_node(target_node_id)
            if not target_node:
                response.success = False
                response.error_message = f"Target node {target_node_id} does not exist"
                break
            unwire_result = target_node.unwire_data(wiring)
            if unwire_result.is_err():
                response.success = False
                response.error_message = (
                    f'Failed to remove wiring "{wiring}": '
                    f"{str(unwire_result.unwrap_err())}"
                )
                break
            successful_unwirings.append(wiring)

        if not response.success:
            # Re-Wire the successful unwirings
            for wiring in successful_unwirings:
                match ros_to_uuid(wiring.target.node_id):
                    case Err(e):
                        response.success = False
                        response.error_message = e
                        return response
                    case Ok(n_id):
                        target_node_id = n_id

                target_node = root.find_node(target_node_id)
                if not target_node:
                    response.success = False
                    response.error_message = (
                        f"Failed to find node: {wiring.target.node_id}"
                    )
                    return response

                wire_data_result = target_node.wire_data(wiring)
                if wire_data_result.is_err():
                    response.success = False
                    response.error_message = (
                        f'Failed to redo wiring "{wiring}": {str(wire_data_result.unwrap_err())}\n'
                        f"Previous error: {response.error_message}"
                    )
                    get_logger("tree_manager").get_child(self.name).error(
                        "Failed to rewire successful unwiring after error. "
                        "Tree is in undefined state!"
                    )
                    return response
            return response
        else:
            # We've removed these NodeDataWirings, so remove them from tree_msg as
            # well.
            for wiring in request.wirings:
                if wiring in self.tree_structure.data_wirings:
                    cast(list, self.tree_structure.data_wirings).remove(wiring)
            self.publish_structure()
        return response

    @typechecked
    def get_subtree(
        self, request: GetSubtree.Request, response: GetSubtree.Response
    ) -> GetSubtree.Response:
        if request.root_id not in self.nodes:
            response.success = False
            response.error_message = f'Node "{request.root_id}" does not exist!'
            return response

        match ros_to_uuid(request.root_id):
            case Err(e):
                response.success = False
                response.error_message = e
                return response
            case Ok(n_id):
                root_id = n_id

        get_subtree_msg_result = self.nodes[root_id].get_subtree_msg()

        if get_subtree_msg_result.is_err():
            response.subtree = False
            response.error_message = (
                "Error retrieving subtree rooted at "
                f"{request.root_id}: {str(get_subtree_msg_result.unwrap_err())}"
            )
            return response

        subtree_msg = get_subtree_msg_result.unwrap()
        response.success = True
        response.subtree = subtree_msg[0]
        return response

    @typechecked
    def generate_subtree(
        self, request: GenerateSubtree.Request, response: GenerateSubtree.Response
    ) -> GenerateSubtree.Response:
        """
        Generate a subtree generated from the provided list of nodes and the loaded tree.

        This also adds all relevant parents to the tree message, resulting in a tree that is
        executable and does not contain any orpahned nodes.
        """
        whole_tree = deepcopy(self.tree_structure)

        root_result = self.find_root()
        if root_result.is_err():
            response.success = False
            response.error_message = (
                "Could not determine tree root: " f"{str(root_result.unwrap_err())}"
            )
            return response
        root = root_result.unwrap()

        if not root:
            response.success = False
            response.error_message = "No tree message available"
            return response
        nodes = set()
        for node in whole_tree.nodes:
            nodes.add(node.node_id)

        nodes_to_keep = set()
        nodes_to_remove = set()
        for node in whole_tree.nodes:
            for search_node in request.node_ids:
                if node.node_id == search_node or search_node in node.child_ids:
                    nodes_to_keep.add(node.node_id)

        for node in nodes:
            if node not in nodes_to_keep:
                nodes_to_remove.add(node)

        manager = TreeManager(
            ros_node=self.ros_node,
            name="temporary_tree_manager",
            debug_manager=DebugManager(ros_node=self.ros_node),
        )

        load_response = LoadTree.Response()
        load_response = manager.load_tree(
            request=LoadTree.Request(tree=whole_tree),
            response=load_response,
        )

        if load_response.success:
            for node_id in nodes_to_remove:
                manager.remove_node(
                    RemoveNode.Request(node_id=node_id, remove_children=False),
                    RemoveNode.Response(),
                )
            root_result = manager.find_root()
            if root_result.is_err():
                response.success = False
                response.error_message = (
                    "Could not determine new subtree root: "
                    f"{str(root_result.unwrap_err())}"
                )
                return response
            root = root_result.unwrap()
            if not root:
                get_logger("tree_manager").get_child(self.name).info("No nodes in tree")
            else:
                manager.tree_structure.root_id = uuid_to_ros(root.node_id)
            response.success = True
            response.tree = manager.structure_to_msg()
            return response
        else:
            response.success = False
            response.error_message = (
                "Could not load tree into the new subtree" + load_response.error_message
            )
            return response

    #########################
    # Service Handlers Done #
    #########################

    @typechecked
    def instantiate_node_from_msg(
        self,
        node_msg: NodeStructure,
        ros_node: rclpy.node.Node,
        permissive: bool = False,
    ) -> Result[Node, BehaviorTreeException]:

        node_result = Node.from_msg(
            node_msg,
            ros_node,
            debug_manager=self.debug_manager,
            subtree_manager=self.subtree_manager,
            permissive=permissive,
        )
        if node_result.is_err():
            get_logger("tree_manager").get_child(self.name).error(
                f"Failed to instanciate node: {node_msg}: {node_result.unwrap_err()}"
            )
            return node_result
        node_instance = node_result.unwrap()

        self.nodes[node_instance.node_id] = node_instance

        return Ok(node_instance)

    def structure_to_msg(self) -> TreeStructure:
        root_result = self.find_root()
        if root_result.is_ok():
            root = root_result.unwrap()
            if root is not None:
                get_subtree_msg_result = root.get_subtree_msg()
                if get_subtree_msg_result.is_err():
                    self.tree_structure.nodes = [
                        node.to_structure_msg() for node in self.nodes.values()
                    ]
                else:
                    subtree = get_subtree_msg_result.unwrap()[0]
                    self.tree_structure.nodes = subtree.nodes
                    self.tree_structure.public_node_data = subtree.public_node_data
            else:
                self.tree_structure.nodes = []
        else:
            get_logger("tree_manager").get_child(self.name).warn(
                f"Strange topology {str(root_result.unwrap_err())}"
            )
            # build a tree_structure out of this strange topology,
            # so the user can fix it in the editor
            self.tree_structure.nodes = [
                node.to_structure_msg() for node in self.nodes.values()
            ]
        return self.tree_structure

    def state_to_msg(self) -> TreeState:
        self.tree_state.state = self.state
        self.tree_state.node_states = [
            node.to_state_msg() for node in self.nodes.values()
        ]
        return self.tree_state

    def data_to_msg(self) -> TreeData:
        self.tree_data.wiring_data = []
        for node in self.nodes.values():
            self.tree_data.wiring_data.extend(node.wire_data_msg_list())
        return self.tree_data


@typechecked
def get_success(response: dict | Any) -> bool:
    if isinstance(response, dict):
        return response["success"]

    return response.success


@typechecked
def get_error_message(response: dict | Any) -> str:
    if isinstance(response, dict):
        return response["error_message"]

    return response.error_message
