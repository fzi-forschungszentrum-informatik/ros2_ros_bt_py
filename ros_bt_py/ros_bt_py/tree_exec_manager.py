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
import os
import uuid
from functools import wraps
from packaging.version import Version
from threading import Thread, Lock, RLock
from typing import Any, Callable, Dict, Optional, List, cast

from ros_bt_py.vendor.result import Err, Ok, Result

import rclpy
from rclpy.utilities import ok
import rclpy.node
from rclpy.duration import Duration

import yaml
import yaml.scanner
from typeguard import typechecked


from ros_bt_py.migrate_tree_files import migrate_legacy_tree_structure
from ros_bt_py.logging_manager import LoggingManager
from ros_bt_py_interfaces.msg import (
    NodeStructure,
    NodeDataLocation,
    TreeStructure,
    TreeStructureList,
    TreeState,
    TreeStateList,
    TreeData,
    TreeDataList,
    Wiring,
)

from ros_bt_py_interfaces.srv import (
    ClearTree,
    ControlTreeExecution,
    LoadTree,
    LoadTreeFromPath,
    MigrateTree,
    ReloadTree,
)

import rosidl_runtime_py
from std_srvs.srv import SetBool

import ament_index_python

from ros_bt_py.data_flow_manager import DataFlowManager
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import (
    BehaviorTreeException,
    MissingParentError,
    TreeTopologyError,
)
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.ros_helpers import ros_to_uuid, uuid_to_ros
from ros_bt_py.node import Node, load_node_module

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float64


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
    def service_handler(self: "TreeExecManager", request: Any, response: Any, **kwds):
        if self.state != TreeState.EDITABLE:
            response.success = False
            response.error_message = (
                f"Cannot edit tree in state {self.state}."
                f"You need to shut down the tree to enable editing."
            )
            return response
        try:
            with self._edit_lock:
                return func(self, request, response, **kwds)
        finally:
            # TODO Find a way to check if we're the outermost level
            #   to reduce repeating publish calls
            self.publish_structure()

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


class TreeExecManager:
    """
    Provide methods to load and run a Behavior Tree.

    These methods are suited (intended, even) for use as ROS service handlers.
    """

    logging_manager: LoggingManager
    data_flow_manager: DataFlowManager
    subtree_manager: SubtreeManager
    debug_manager: DebugManager

    def __init__(
        self,
        ros_node: rclpy.node.Node,
        tree_id: uuid.UUID = uuid.UUID(int=0),
        name: str = "UNKNOWN TREE",
        module_list: Optional[List[str]] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        logging_manager: Optional[LoggingManager] = None,
        data_flow_manager: Optional[DataFlowManager] = None,
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

        self._tree_structure = TreeStructure()
        # This reassignment makes the typing happy,
        #   because they ensure that `.append .extent .remove ...` exists
        self._tree_structure.data_wirings = []

        self._tree_state = TreeState()

        self._tree_data = TreeData()
        self.enable_publish_data = False

        self.diagnostic_array = DiagnosticArray()
        self.diagnostic_status = DiagnosticStatus()
        self.diagnostic_array.status = [self.diagnostic_status]

        if logging_manager is None:
            self.logging_manager = LoggingManager(ros_node=self.ros_node)
        else:
            self.logging_manager = logging_manager

        if data_flow_manager is None:
            self.get_logger().info(
                "Tree manager instantiated without explicit data flow manager "
                "- building our own with default parameters",
                internal=True,
            )
            self.data_flow_manager = DataFlowManager()
        else:
            self.data_flow_manager = data_flow_manager

        if subtree_manager is None:
            self.get_logger().info(
                "Tree manager instantiated without explicit subtree manager "
                "- building our own with default parameters",
                internal=True,
            )
            self.subtree_manager = SubtreeManager()
        else:
            self.subtree_manager = subtree_manager

        self.publish_tree_structure = publish_tree_structure_callback
        if self.publish_tree_structure is None:
            self.get_logger().info(
                "No callback for publishing tree structure provided",
                internal=True,
            )
        self.publish_tree_state = publish_tree_state_callback
        if self.publish_tree_state is None:
            self.get_logger().info(
                "No callback for publishing tree state provided",
                internal=True,
            )
        self.publish_tree_data = publish_tree_data_callback
        if self.publish_tree_data is None:
            self.get_logger().info(
                "No callback for publishing tree data provided",
                internal=True,
            )

        self.publish_diagnostic = publish_diagnostic_callback
        if self.publish_diagnostic is None:
            self.get_logger().info(
                "No callback for publishing node diagnostics provided", internal=True
            )

        self.publish_tick_frequency = publish_tick_frequency_callback
        if self.publish_tick_frequency is None:
            self.get_logger().info(
                "No callback for publishing tree frequency provided",
                internal=True,
            )

        if debug_manager is None:
            self.get_logger().info(
                "Tree manager instantiated without explicit debug manager "
                "- building our own with default parameters",
                internal=True,
            )
            self.debug_manager = DebugManager(ros_node=self.ros_node)
        else:
            self.debug_manager = debug_manager

        self._tree_lock = Lock()
        # Initialized ROS messages and component managers, properties should be safe now

        self.tree_id = tree_id
        self.name = name
        self.state = TreeState.EDITABLE
        self.tick_frequency_hz = tick_frequency_hz

        self.rate = self.ros_node.create_rate(self.tick_frequency_hz)

        self.tick_sliding_window = [self.tick_frequency_hz] * 10

        self.nodes: Dict[uuid.UUID, Node] = {}

        self._edit_lock = RLock()
        # Stop the tick thread after a single tick
        self._once: bool = False
        # Stop the tick thread after the tree returns something other than
        # RUNNING for the first time
        self._stop_after_result: bool = False

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
    @typechecked
    def tree_id(self) -> Result[uuid.UUID, str]:
        return ros_to_uuid(self._tree_structure.tree_id)

    @tree_id.setter
    @typechecked
    def tree_id(self, new_tree_id: uuid.UUID) -> None:
        with self._tree_lock:
            self._tree_structure.tree_id = uuid_to_ros(new_tree_id)
            self._tree_state.tree_id = uuid_to_ros(new_tree_id)
            self._tree_data.tree_id = uuid_to_ros(new_tree_id)
            self.logging_manager.set_tree_id(new_tree_id)

    @property
    @typechecked
    def name(self) -> str:
        return self._tree_structure.name

    @name.setter
    @typechecked
    def name(self, new_name: str) -> None:
        with self._tree_lock:
            self._tree_structure.name = new_name
            self.logging_manager.set_tree_name(new_name)

    @property
    @typechecked
    def root_id(self) -> Result[uuid.UUID, str]:
        return ros_to_uuid(self._tree_structure.root_id)

    @root_id.setter
    @typechecked
    def root_id(self, new_root_id: uuid.UUID) -> None:
        with self._tree_lock:
            self._tree_structure.root_id = uuid_to_ros(new_root_id)

    @property
    @typechecked
    def state(self) -> str:
        return self._tree_state.state

    @state.setter
    @typechecked
    def state(self, new_state: str) -> None:
        with self._tree_lock:
            self._tree_state.state = new_state
            self.ros_node.get_logger().debug(
                f"Updating tree state to {self._tree_state.state}"
            )

    @property
    @typechecked
    def tick_frequency_hz(self) -> float:
        return self._tree_structure.tick_frequency_hz

    @tick_frequency_hz.setter
    @typechecked
    def tick_frequency_hz(self, frequency: float) -> None:
        with self._tree_lock:
            if frequency == 0.0:
                self.get_logger().warn(
                    f"Tick frequency of {frequency} is invalid. Setting to 10.0"
                )
                frequency = 10.0
            self._tree_structure.tick_frequency_hz = frequency

    @property
    @typechecked
    def wirings(self) -> list[Wiring]:
        return cast(list, self._tree_structure.data_wirings)

    @wirings.setter
    @typechecked
    def wirings(self, wirings: list[Wiring]) -> None:
        self._tree_structure.data_wirings = wirings

    def get_logger(self) -> LoggingManager:
        return self.logging_manager

    def set_diagnostics_name(self) -> None:
        """
        Set the tree name for ROS diagnostics.

        If the BT has a name, this name will published in diagnostics.
        Otherwise, the root name of the tree is used.
        """
        if self.name:
            self.diagnostic_status.name = os.path.splitext(self.name)[0]
            return

        match self.root_id:
            case Ok(r_id):
                root = self.nodes.get(r_id)
        if root is not None:
            self.get_logger().warn(
                "No tree name was found. Diagnostics data from the behavior tree will be"
                f"published under the name of the root_node: {self.diagnostic_status.name}",
                internal=True,
            )
            return

        self.diagnostic_status.name = ""
        self.get_logger().warn(
            "Neither a tree name nor the name from the root_node was found."
            "Diagnostics data from the behavior tree will be "
            "published without further name specifications",
            internal=True,
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
                    f'Tree "{self.name}" has multiple nodes without parents.'
                )
            )
        if not possible_roots:
            return Err(
                TreeTopologyError(
                    f'All nodes in tree "{self.name} have parents. You have '
                    "made a cycle, which makes the tree impossible to run!"
                )
            )
        self.root_id = possible_roots[0].node_id
        return Ok(possible_roots[0])

    def tick_report_exceptions(self) -> None:
        """Wrap :meth:`TreeManager.tick()` and catch *all* errors."""
        tick_result = self.tick()

        if tick_result.is_err():
            self.get_logger().error(
                f"Encountered error while ticking tree: {tick_result.unwrap_err()}"
            )
            self._last_error = f"{tick_result.unwrap_err()}"
            self.state = TreeState.ERROR
            self.publish_state()

    @typechecked
    def tick(
        self,
    ) -> Result[None, BehaviorTreeException]:
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
            self.get_logger().error("Could not find tree root!")
            return Err(root_result.unwrap_err())

        root = root_result.unwrap()
        if not root:
            self.get_logger().info("No nodes in tree, tick will not do anything")
            return Err(
                TreeTopologyError("No nodes in  the tree, tick will do nothing!")
            )

        if root.state in (BTNodeState.UNINITIALIZED, BTNodeState.SHUTDOWN):
            # TODO This is a very ugly position to put this,
            #   we should restructure execution actions and commands.
            self.data_flow_manager.initialize(self.nodes, self.wirings)
            root.setup()
            if root.state is not BTNodeState.IDLE:
                self.state = TreeState.ERROR
                self.publish_state()
                return Err(BehaviorTreeException("Tree not in idle state after setup!"))

        while True:
            tick_start_timestamp = self.ros_node.get_clock().now()
            if self.state == TreeState.STOP_REQUESTED:
                break

            match self.data_flow_manager.push_incoming_data():
                case Err(e):
                    self.get_logger().error(f"Pushing tree inputs failed: {e}")
                    return Err(BehaviorTreeException(e))
                case Ok(None):
                    pass

            tick_result = root.tick()

            if tick_result.is_err():
                tick_err = tick_result.unwrap_err()
                self.get_logger().error(f"Ticking the tree failed: {tick_err}")
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
            tick_rate = self.tick_frequency_hz

            if (1 / tick_rate) > (duration.nanoseconds * 1e9):
                self.get_logger().warn(
                    "Tick took longer than set period, cannot tick at "
                    f"{self.tick_frequency_hz:.2f} Hz"
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
            self.get_logger().warn(f"Could not find root {root_result.unwrap_err()}")
            response.error_message = str(root_result.unwrap_err())
            response.success = False
            return response
        root = root_result.unwrap()
        if not root:
            # No root, no problems
            response.success = True
            return response
        if not (root.state in [BTNodeState.UNINITIALIZED, BTNodeState.SHUTDOWN]):
            self.get_logger().error("Please shut down the tree before clearing it")
            response.success = False
            response.error_message = "Please shut down the tree before clearing it"
            return response

        self.nodes = {}
        with self._tree_lock:
            # These reassignments makes the typing happy,
            #   because they ensure that `.append .extent .remove ...` exists
            self.wirings = []
            self.name = "UNKNOWN TREE"
            self._tree_structure.path = ""
        self.subtree_manager.clear_subtrees()
        self.clear_diagnostics_name()
        response.success = True
        return response

    @is_edit_service
    @typechecked
    def reload_tree(
        self, request: Optional[ReloadTree.Request], response: ReloadTree.Response
    ) -> ReloadTree.Response:
        """Reload the currently loaded tree."""
        load_response = LoadTree.Response()
        load_response = self.load_tree(
            request=LoadTree.Request(tree=self.structure_to_msg()),
            response=load_response,
        )

        response.success = load_response.success
        response.error_message = load_response.error_message

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

        # Clear existing tree, then replace it with the message's contents
        self.clear(None, ClearTree.Response())
        # First just add all nodes to the tree, then restore tree structure
        for node in tree.nodes:
            match self.instantiate_node_from_msg(
                node_msg=node,
                ros_node=self.ros_node,
            ):
                case Err(e):
                    response.success = False
                    response.error_message = str(e)
                    return response
                case Ok(n):
                    self.nodes[n.node_id] = n

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
        updated_wirings = []
        for wiring in tree.data_wirings:
            match self.validate_wiring(wiring):
                case Err(e):
                    response.success = False
                    response.error_message = str(e)
                    return response
                case Ok(None):
                    updated_wirings.append(wiring)

        self.name = tree.name
        self._tree_structure.path = tree.path
        self.tick_frequency_hz = tree.tick_frequency_hz

        # These reassignments makes the typing happy,
        #   because they ensure that `.append .extent .remove ...` exists
        self._tree_structure.data_wirings = updated_wirings
        self._tree_structure.public_inputs = list(tree.public_inputs)
        self._tree_structure.public_outputs = list(tree.public_outputs)

        self.rate = self.ros_node.create_rate(frequency=self.tick_frequency_hz)

        # Clear state and data
        self._tree_state.node_states = []
        self._tree_data.wiring_data = []

        # find and set root name
        self.find_root()

        response.success = True
        self.get_logger().info("Successfully loaded tree")
        if self.publish_diagnostic is None:
            self.set_diagnostics_name()
        return response

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
            self.get_logger().info("Shutting down a tree with no nodes.")
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
            self.get_logger().warn(response.error_message)
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
            self._tick_thread.join((1.0 / self.tick_frequency_hz) * 4.0)
            # If we're debugging or setting up (and ROS is not
            # shutting down), keep sleepin until the thread
            # finishes
            while self._tick_thread.is_alive() and ok():
                self._tick_thread.join((1.0 / self.tick_frequency_hz) * 4.0)
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
                self.get_logger().error(response.error_message)
            else:
                response.error_message = (
                    f"Successfully stopped ticking, but tree state "
                    f"is {state_after_joining}, not IDLE"
                )
                response.success = False
                self.get_logger().error(response.error_message)
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
            self.get_logger().warn(response.error_message)
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
                self.tick_frequency_hz = request.tick_frequency_hz
                self.rate = self.ros_node.create_rate(frequency=self.tick_frequency_hz)
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
            self.get_logger().warn(response.error_message)
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
                self.get_logger().info("Resetting a tree with no root.")
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
                        "Tried to join tick thread with Tree state IDLE, but failed!"
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
                self.get_logger().warn(response.error_message)
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
                    self._tick_thread.join((1.0 / self.tick_frequency_hz) * 4.0)

                    # If we're debugging or setting up (and ROS is not
                    # shutting down), keep sleeping until the thread
                    # finishes
                    while self._tick_thread.is_alive() and ok():
                        self._tick_thread.join((1.0 / self.tick_frequency_hz) * 4.0)
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
                    self.get_logger().error(response.error_message)
                    return response
                else:
                    response.error_message = (
                        f"Successfully stopped ticking, but tree state is "
                        f"{state_after_joining}, not IDLE"
                    )
                    response.success = False
                    self.get_logger().error(response.error_message)
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
                        self.get_logger().error(
                            f"Root node ({str(root)}) state after unticking is neither "
                            f"IDLE nor PAUSED, but {state}"
                        )
                        response.error_message = "Failed to untick root node."
                        return response
                else:
                    self.get_logger().info("Unticking a tree with no nodes.")
                    response.tree_state = TreeState.IDLE
                    response.success = True

            else:
                self.get_logger().info(
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
            self.get_logger().info("Doing nothing in this request")
            response.success = True
        else:
            response.error_message = f"Received unknown command {request.command}"
            self.get_logger().error(response.error_message)
            response.success = False

        return response

    #########################
    # Service Handlers Done #
    #########################

    @typechecked
    def instantiate_node_from_msg(
        self,
        node_msg: NodeStructure,
        ros_node: rclpy.node.Node,
    ) -> Result[Node, BehaviorTreeException]:

        node_result = Node.from_msg(
            node_msg,
            ros_node,
            debug_manager=self.debug_manager,
            subtree_manager=self.subtree_manager,
            logging_manager=self.get_logger(),
            data_flow_manager=self.data_flow_manager,
        )
        if node_result.is_err():
            self.get_logger().error(
                f"Failed to instanciate node: {node_msg}: {node_result.unwrap_err()}"
            )
            return node_result
        node_instance = node_result.unwrap()

        self.nodes[node_instance.node_id] = node_instance

        return Ok(node_instance)

    def validate_wiring(
        self, wiring_msg: Wiring
    ) -> Result[None, BehaviorTreeException]:
        match ros_to_uuid(wiring_msg.source.node_id):
            case Err(e):
                return Err(BehaviorTreeException(e))
            case Ok(n_id):
                source_node_id = n_id
        match ros_to_uuid(wiring_msg.target.node_id):
            case Err(e):
                return Err(BehaviorTreeException(e))
            case Ok(n_id):
                target_node_id = n_id
        source_node = self.nodes.get(source_node_id)
        if source_node is None:
            return Err(
                BehaviorTreeException(
                    f"Source node ({source_node_id}) doesn't exist in tree"
                )
            )
        target_node = self.nodes.get(target_node_id)
        if target_node is None:
            return Err(
                BehaviorTreeException(
                    f"Target node ({target_node_id}) doesn't exist in tree"
                )
            )
        source_container = source_node.node_config.outputs.get(
            wiring_msg.source.data_key
        )
        if source_container is None:
            return Err(
                BehaviorTreeException(
                    f"Source node ({source_node_id}) doesn't have "
                    f"an output key {wiring_msg.source.data_key}"
                )
            )
        target_container = target_node.node_config.inputs.get(
            wiring_msg.target.data_key
        )
        if target_container is None:
            return Err(
                BehaviorTreeException(
                    f"Target node ({target_node_id}) doesn't have "
                    f"an output key {wiring_msg.target.data_key}"
                )
            )
        if not target_container.get_runtime_type().is_compatible(
            source_container.get_runtime_type()
        ):
            return Err(
                BehaviorTreeException(
                    f"The IO types of source {source_container} "
                    f"and target {target_container} are incompatible"
                )
            )
        return Ok(None)

    def structure_to_msg(self) -> TreeStructure:
        root_result = self.find_root()
        if root_result.is_ok():
            root = root_result.unwrap()
            if root is not None:
                get_subtree_msg_result = root.get_subtree_msg()
                if get_subtree_msg_result.is_err():
                    self._tree_structure.nodes = [
                        node.to_structure_msg() for node in self.nodes.values()
                    ]
                else:
                    subtree = get_subtree_msg_result.unwrap()[0]
                    self._tree_structure.nodes = subtree.nodes
                    self._tree_structure.public_inputs = subtree.public_inputs
                    self._tree_structure.public_outputs = subtree.public_outputs
            else:
                self._tree_structure.nodes = []
        else:
            self.get_logger().warn(f"Strange topology {str(root_result.unwrap_err())}")
            # build a tree structure out of this strange topology,
            # so the user can fix it in the editor
            self._tree_structure.nodes = [
                node.to_structure_msg() for node in self.nodes.values()
            ]
        return self._tree_structure

    def state_to_msg(self) -> TreeState:
        self._tree_state.node_states = [
            node.to_state_msg() for node in self.nodes.values()
        ]
        return self._tree_state

    def data_to_msg(self) -> TreeData:
        self._tree_data.wiring_data = self.data_flow_manager.get_wiring_data()
        return self._tree_data
