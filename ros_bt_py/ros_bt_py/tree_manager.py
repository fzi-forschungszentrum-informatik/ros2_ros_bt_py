# Copyright 2023 FZI Forschungszentrum Informatik

import inspect
import os
import traceback
from copy import deepcopy
from functools import wraps
from threading import Thread, Lock, RLock
from typing import Callable, Optional, List

import rclpy
import rclpy.node
from rclpy.duration import Duration
from rclpy.logging import get_logger

import yaml
import yaml.scanner
from typeguard import typechecked

from ros_bt_py_interfaces.msg import (
    DebugInfo,
    DebugSettings,
    NodeDiagnostics,
    Tree,
    Node as NodeMsg,
    DocumentedNode,
    NodeData,
    NodeDataLocation,
)
from ros_bt_py_interfaces.srv import (
    AddNode,
    AddNodeAtIndex,
    ChangeTreeName,
    ClearTree,
    Continue,
    ControlTreeExecution,
    FixYaml,
    GenerateSubtree,
    GetAvailableNodes,
    GetSubtree,
    LoadTree,
    LoadTreeFromPath,
    MigrateTree,
    ModifyBreakpoints,
    MorphNode,
    MoveNode,
    ReloadTree,
    RemoveNode,
    ReplaceNode,
    SetExecutionMode,
    SetOptions,
    SetSimulateTick,
    WireNodeData,
)

import ament_index_python

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.exceptions import (
    BehaviorTreeException,
    MissingParentError,
    TreeTopologyError,
)
from ros_bt_py.helpers import (
    fix_yaml,
    remove_input_output_values,
    json_encode,
    json_decode,
)
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
    def service_handler(self, request, response, **kwds):
        tree_state = self.get_state()
        if tree_state != Tree.EDITABLE:
            return {
                "success": False,
                "error_message": f"Cannot edit tree in state {tree_state}."
                f"You need to shut down the tree to enable editing.",
            }
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
            # remove option wirings
            if "nodes" in datum:
                for node in datum["nodes"]:
                    node.pop("option_wirings", None)
            response.tree = message_converter.convert_dictionary_to_ros_message(
                Tree, datum
            )
            read_data = True
        else:
            response.success = False
            response.error_message = (
                "Tree YAML file must contain " "exactly one YAML object!"
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
    file_name = ""
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
            file_name = os.path.basename(tree_file.name)
            tree_yaml = tree_file.read()
            try:
                response = parse_tree_yaml(tree_yaml=tree_yaml)
            except yaml.scanner.ScannerError as ex:
                get_logger("tree_manager").warn(
                    f"Encountered a ScannerError while parsing the tree yaml: {str(ex)}\n"
                    f"This is most likely caused by a tree that was created with "
                    f"PyYAML 5 and genpy < 0.6.10.\n"
                    f"Attempting to fix this automatically..."
                )
                # ScannerError most likely means that the tree was created
                # with PyYAML 5 and genpy <0.6.10
                # fix this by correctly indenting the broken lists
                fix_yaml_response = fix_yaml(
                    request=FixYaml.Request(broken_yaml=tree_yaml)
                )

                # try parsing again with fixed tree_yaml:
                response = parse_tree_yaml(tree_yaml=fix_yaml_response.fixed_yaml)
            # remove input and output values from nodes
            tree = remove_input_output_values(tree=response.tree)

    tree.name = file_name

    response.success = True
    response.tree = tree
    return response


@typechecked
def get_available_nodes(
    request: GetAvailableNodes.Request, response: GetAvailableNodes.Response
) -> Optional[GetAvailableNodes.Response]:
    """
    List the types of nodes that are currently known.

    This includes all nodes from modules that were passed to our
    constructor in `module_list`, ones from modules that nodes have
    been successfully loaded from since launch, and ones from
    modules explicitly asked for in `request.node_modules`

    :param ros_bt_py_msgs.srv.GetAvailableNodesRequest request:

    If `request.node_modules` is not empty, try to load those
    modules before responding.

    :returns: :class:`ros_bt_py_msgs.src.GetAvailableNodesResponse` or `None`
    """
    for module_name in request.node_modules:
        if module_name and load_node_module(module_name) is None:
            response.success = False
            response.error_message = f"Failed to import module {module_name}"
            return response

    def to_node_data(data_map):
        return [
            NodeData(key=name, serialized_value=json_encode(type_or_ref))
            for (name, type_or_ref) in data_map.items()
        ]

    for (module, nodes) in Node.node_classes.items():
        for (class_name, node_classes) in nodes.items():
            for node_class in node_classes:
                max_children = node_class._node_config.max_children
                max_children = -1 if max_children is None else max_children
                doc = inspect.getdoc(node_class) or ""
                response.available_nodes.append(
                    DocumentedNode(
                        module=module,
                        node_class=class_name,
                        version=node_class._node_config.version,
                        max_children=max_children,
                        name=class_name,
                        options=to_node_data(node_class._node_config.options),
                        inputs=to_node_data(node_class._node_config.inputs),
                        outputs=to_node_data(node_class._node_config.outputs),
                        doc=str(doc),
                        tags=node_class._node_config.tags,
                    )
                )

    response.success = True
    return response


@typechecked
class TreeManager:
    """
    Provide methods to manage a Behavior Tree.

    These methods are suited (intended, even) for use as ROS service handlers.
    """

    def __init__(
        self,
        ros_node: rclpy.node.Node,
        name: Optional[str] = None,
        module_list: Optional[List[str]] = None,
        debug_manager: Optional[DebugManager] = None,
        tick_frequency_hz: float = 10.0,
        publish_tree_callback: Optional[Callable[[Tree], None]] = None,
        publish_debug_info_callback: Optional[Callable[[DebugInfo], None]] = None,
        publish_debug_settings_callback: Optional[
            Callable[[DebugSettings], None]
        ] = None,
        publish_node_diagnostics_callback: Optional[
            Callable[[NodeDiagnostics], None]
        ] = None,
        publish_diagnostic_callback: Optional[Callable[[DiagnosticArray], None]] = None,
        publish_tick_frequency_callback: Optional[Callable[[Float64], None]] = None,
        diagnostics_frequency: float = 1.0,
        show_traceback_on_exception: bool = False,
        simulate_tick: bool = False,
        succeed_always: bool = False,
    ) -> None:
        self.ros_node = ros_node
        self.name = name
        self.publish_tree = publish_tree_callback
        if self.publish_tree is None:
            get_logger("tree_manager").info(
                "No callback for publishing tree data provided."
            )

        self.publish_debug_info = publish_debug_info_callback
        if self.publish_debug_info is None:
            get_logger("tree_manager").info(
                "No callback for publishing debug data provided."
            )

        self.publish_debug_settings = publish_debug_settings_callback
        if self.publish_debug_settings is None:
            get_logger("tree_manager").info(
                "No callback for publishing debug data provided."
            )

        self.publish_node_diagnostics = publish_node_diagnostics_callback
        if self.publish_node_diagnostics is None:
            get_logger("tree_manager").info(
                "No callback for publishing node diagnostics data provided."
            )

        self.publish_diagnostic = publish_diagnostic_callback
        if self.publish_diagnostic is None:
            get_logger("tree_manager").info(
                "No callback for publishing node diagnostics provided"
            )

        self.publish_tick_frequency = publish_tick_frequency_callback
        if self.publish_tick_frequency is None:
            get_logger("tree_manager").info(
                "No callback for publishing tree frequency provided"
            )

        self.debug_manager = debug_manager
        if not self.debug_manager:
            get_logger("tree_manager").info(
                "Tree manager instantiated without explicit debug manager "
                "- building our own with default parameters"
            )
            self.debug_manager = DebugManager(ros_node=self.ros_node)

        self.show_traceback_on_exception = show_traceback_on_exception

        # self.capability_interfaces_callback = capability_interfaces_callback
        # if self.capability_interfaces_callback is not None:
        #    get_logger("tree_manager").info(
        #        "No callback for local capability interface announcements provided."
        #    )

        # When auto succeed nodes is set, all nodes will return the succeeded status
        self.succeed_always = succeed_always

        # The simulate ticks flag requires nodes that call external services or cause changes to
        # the environment to skip these calls.
        self.simulate_tick = simulate_tick

        if name is None:
            name = ""

        if tick_frequency_hz == 0.0:
            tick_frequency_hz = 10.0

        self.tick_sliding_window = [tick_frequency_hz] * 10

        self.debug_manager.publish_debug_info = self.publish_info
        self.debug_manager.publish_debug_settings = self.publish_debug_settings
        self.debug_manager.publish_node_diagnostics = self.publish_node_diagnostics

        self.nodes = {}

        self._state_lock = Lock()
        self._edit_lock = RLock()

        self._setting_up = False
        # Stop the tick thread after a single tick
        self._once = False
        # Stop the tick thread after the tree returns something other than
        # RUNNING for the first time
        self._stop_after_result = False

        self.tree_msg = Tree()
        self.tree_msg.name = name if name else ""
        self.tree_msg.tick_frequency_hz = tick_frequency_hz
        self.rate = self.ros_node.create_rate(self.tree_msg.tick_frequency_hz)

        self.diagnostic_array = DiagnosticArray()
        self.diagnostic_status = DiagnosticStatus()
        self.diagnostic_array.status = [self.diagnostic_status]

        self._last_error = None
        with self._state_lock:
            self.tree_msg.state = Tree.EDITABLE

        self._tick_thread = None

        # Skip if module_list is empty or None
        if module_list:
            for module_name in module_list:
                load_node_module(module_name)

        self.publish_info(self.debug_manager.get_debug_info_msg())

        if self.publish_diagnostic is not None:
            self.ros_node.create_timer(
                1.0 / diagnostics_frequency, self.diagnostic_callback
            )

    def get_state(self):
        with self._state_lock:
            return self.tree_msg.state

    def set_diagnostics_name(self):
        """
        Set the tree name for ROS diagnostics.

        If the BT has a name, this name will published in diagnostics.
        Otherwise, the root name of the tree is used.
        """
        if self.tree_msg.name:
            self.diagnostic_status.name = os.path.splitext(self.tree_msg.name)[0]
        elif self.tree_msg.root_name:
            self.diagnostic_status.name = self.tree_msg.root_name
            get_logger("tree_manager").warn(
                "No tree name was found. Diagnostics data from the behavior tree will be"
                f"published under the name of the root_node: {self.tree_msg.root_name}"
            )
        else:
            self.diagnostic_status.name = ""
            get_logger("tree_manager").warn(
                "Neither a tree name nor the name from the root_node was found."
                "Diagnostics data from the behavior tree will be "
                "published without further name specifications"
            )

    def clear_diagnostics_name(self):
        """Clear the name for ROS diagnostics."""
        self.diagnostic_status.name = ""

    def diagnostic_callback(self):
        if self.publish_diagnostic is None:
            return
        if self.get_state() == Tree.TICKING:
            self.diagnostic_status.level = 0
            self.diagnostic_status.message = "Ticking"
            # self.tick_stat.values = [KeyValue(key = 'Ticking', value = 'True')]
        elif self.get_state() in (
            Tree.EDITABLE,
            Tree.IDLE,
            Tree.WAITING_FOR_TICK,
            Tree.STOP_REQUESTED,
        ):
            self.diagnostic_status.level = 1
            self.diagnostic_status.message = "Not ticking"
            # self.tick_stat.values = [KeyValue(key = 'Ticking', value = 'False')]
        elif self.get_state() == Tree.ERROR:
            self.diagnostic_status.level = 2
            self.diagnostic_status.message = "Error in Behavior Tree"
        self.publish_diagnostic(self.diagnostic_array)

    def publish_info(self, debug_info_msg: Optional[DebugInfo] = None, ticked=False):
        """
        Publish the current tree state using the callback supplied to the constructor.

        In most cases, you'll want that callback to publish to a ROS
        topic.

        If debugging is enabled, also publish debug info.
        """
        if self.publish_tree:
            self.publish_tree(self.to_msg(ticked=ticked))
        if debug_info_msg and self.publish_debug_info:
            self.publish_debug_info(debug_info_msg)

    def find_root(self) -> Optional[Node]:
        """
        Find the root node of the tree.

        :raises: `TreeTopologyError`

        if nodes exist, but either no root or multiple roots are
        found.
        """
        if not self.nodes:
            return None
        # Find root node
        possible_roots = [node for node in self.nodes.values() if not node.parent]

        if len(possible_roots) > 1:
            raise TreeTopologyError(
                f'Tree "{self.tree_msg.name}" has multiple nodes without parents.'
            )
        if not possible_roots:
            raise TreeTopologyError(
                f'All nodes in tree "{self.tree_msg.name} have parents. You have '
                "made a cycle, which makes the tree impossible to run!"
            )
        return possible_roots[0]

    def tick_report_exceptions(self):
        """Wrap :meth:`TreeManager.tick()` and catch *all* errors."""
        try:
            self.tick()
        except Exception as ex:
            get_logger("tree_manager").error(
                f"Encountered error while ticking tree: {ex}, {traceback.format_exc()}"
            )
            with self._state_lock:
                if self.show_traceback_on_exception:
                    self._last_error = f"{ex}, {traceback.format_exc()}"
                else:
                    self._last_error = f"{ex}"

                self.tree_msg.state = Tree.ERROR

    def tick(self, once=None):
        """
        Execute a tick, starting from the tree's root.

        This behaves differently based on the current configuration of
        the `TreeManager` - it can tick once, continuously, until the
        tree reports a result (either SUCCEEDED or FAILED).

        Each of those can be done in debug mode (i.e. requiring a call
        of :meth:`TreeManager.debug_step()` after each node's tick)
        too.

        This method should *NOT* be called directly, but rather
        triggered via :meth:`TreeManager.control_execution()`!
        """
        if once is not None:
            self._once = once

        # First check for nodes with missing parents
        orphans = [
            f'"{node.name}"(parent: {node.parent.name if node.parent else ""}")'
            for node in self.nodes.values()
            if node.parent and node.parent.name not in self.nodes
        ]
        if orphans:
            raise MissingParentError(
                f'The following nodes\' parents are missing: {", ".join(orphans)}'
            )
        root = self.find_root()
        if not root:
            get_logger("tree_manager").info(
                "No nodes in tree, tick will not do anything"
            )
            return
        with self._state_lock:
            self.tree_msg.root_name = root.name
        if root.state in (NodeMsg.UNINITIALIZED, NodeMsg.SHUTDOWN):
            with self._state_lock:
                self._setting_up = True
            root.setup()
            with self._state_lock:
                self._setting_up = False
        while True:
            tick_start_timestamp = self.ros_node.get_clock().now()
            if self.get_state() == Tree.STOP_REQUESTED:
                break
            root.tick()
            self.publish_info(self.debug_manager.get_debug_info_msg(), ticked=True)

            if self._stop_after_result:
                if root.state in [NodeMsg.FAILED, NodeMsg.SUCCEEDED]:
                    break

            if self._once:
                # Return immediately, not unticking anything
                self._once = False
                with self._state_lock:
                    self.tree_msg.state = Tree.WAITING_FOR_TICK
                return
            tick_end_timestamp = self.ros_node.get_clock().now()

            duration: Duration = tick_end_timestamp - tick_start_timestamp
            tick_rate = self.tree_msg.tick_frequency_hz

            if (1 / tick_rate) > (duration.nanoseconds * 1e9):
                get_logger("tree_manager").warn(
                    "Tick took longer than set period, cannot tick at "
                    f"{self.tree_msg.tick_frequency_hz:.2f} Hz"
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

        with self._state_lock:
            self.tree_msg.state = Tree.IDLE
        self.publish_info(self.debug_manager.get_debug_info_msg(), ticked=True)

        # Ensure all nodes are stopped and not doing anything in
        # the background.
        root.untick()

    def find_nodes_in_cycles(self):
        """Return a list of all nodes in the tree that are part of cycles."""
        safe_node_names = []
        nodes_in_cycles = []
        # Follow the chain of parent nodes for each node name in self.nodes
        for starting_name in self.nodes:
            cycle_candidates = [starting_name]
            current_node = self.nodes[starting_name]
            while current_node.parent:
                current_node = self.nodes[current_node.parent.name]
                cycle_candidates.append(current_node.name)
                if current_node.name == starting_name:
                    nodes_in_cycles.extend(cycle_candidates)
                    break
                if current_node.name in safe_node_names:
                    # We've already checked for cycles from the parent node, no
                    # need to do that again.
                    safe_node_names.extend(cycle_candidates)
                    break
            if not current_node.parent:
                safe_node_names.extend(cycle_candidates)

        return nodes_in_cycles

    ####################
    # Service Handlers #
    ####################

    @is_edit_service
    def clear(
        self, request: Optional[ClearTree.Request], response: ClearTree.Response
    ) -> ClearTree.Response:
        response.success = True
        try:
            root = self.find_root()
            if not root:
                # No root, no problems
                return response
            if not (root.state in [NodeMsg.UNINITIALIZED, NodeMsg.SHUTDOWN]):
                get_logger("tree_manager").error(
                    "Please shut down the tree before clearing it"
                )
                response.success = False
                response.error_message = "Please shut down the tree before clearing it"
                return response
        except TreeTopologyError as exc:
            get_logger("tree_manager").warn(f"Could not find root {exc}")

        self.nodes = {}
        with self._state_lock:
            self.tree_msg = Tree(
                name="",
                state=Tree.EDITABLE,
                tick_frequency_hz=self.tree_msg.tick_frequency_hz,
            )

        self.publish_info(self.debug_manager.get_debug_info_msg())
        self.clear_diagnostics_name()
        return response

    @is_edit_service
    def load_tree_from_path(
        self, request: LoadTreeFromPath.Request, response: LoadTreeFromPath.Response
    ) -> LoadTreeFromPath.Response:
        """Wrap around load_tree for convenience."""
        tree = Tree()
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
    def load_tree(  # noqa: C901
        self, request: LoadTree.Request, response: LoadTree.Response, prefix=None
    ) -> LoadTree.Response:
        """
        Load a tree from the given message (which may point to a file).

        :param ros_bt_py_msgs.srv.LoadTree request:

        `request.tree` describes the tree to be loaded, including
        nodes, wirings and public node data.

        If the `Tree` message itself isn't populated, but contains a
        `path` to load a tree from, we open the file it points to and
        load that.

        :param str prefix:

        If set, all node names in the tree will be prefixed with this string.

        This is used by the subtree node (using its own name as a
        prefix, since that must be unique in the tree) to ensure
        unique node names for easier debugging.

        """
        if prefix is None:
            prefix = ""

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

        # we should have a tree message with all the info we need now
        # prefix all the node names, if prefix is not the empty string
        if prefix != "":
            tree.name = prefix + tree.name
            for node in tree.nodes:
                node.name = prefix + node.name
                node.child_names = [
                    prefix + child_name for child_name in node.child_names
                ]
            for wiring in tree.data_wirings:
                wiring.source.node_name = prefix + wiring.source.node_name
                wiring.target.node_name = prefix + wiring.target.node_name
            for public_datum in tree.public_node_data:
                public_datum.node_name = prefix + public_datum.node_name

        for public_datum in tree.public_node_data:
            if public_datum.data_kind == NodeDataLocation.OPTION_DATA:
                response.success = False
                response.error_message = (
                    "public_node_data: option values cannot be public!"
                )

                return response

        # Clear existing tree, then replace it with the message's contents
        self.clear(None, ClearTree.Response())
        # add nodes whose children exist already, until all nodes are there
        while len(self.nodes) != len(tree.nodes):
            added = 0
            # find nodes whose children are all in the tree already, then add them
            for node in (
                node
                for node in tree.nodes
                if (
                    node.name not in self.nodes
                    and all((name in self.nodes for name in node.child_names))
                )
            ):
                try:
                    instance = self.instantiate_node_from_msg(
                        node, allow_rename=False, permissive=request.permissive
                    )

                    instance.simulate_tick = self.simulate_tick
                    instance.succeed_always = self.succeed_always

                    for child_name in node.child_names:
                        instance.add_child(self.nodes[child_name])
                    self.nodes[node.name] = instance
                    added += 1
                except BehaviorTreeException as exc:
                    response.success = False
                    response.error_message = str(exc)
                    return response
            if added == 0:
                response.success = False
                response.error_message = "Unable to add all nodes to tree."
                return response

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
            if wiring in self.tree_msg.data_wirings:
                updated_wirings.append(wiring)

        tree.data_wirings = updated_wirings

        self.tree_msg = tree
        if self.tree_msg.tick_frequency_hz == 0.0:
            get_logger("tree_manager").warn(
                "Tick frequency of loaded tree is 0, defaulting to 10Hz"
            )
            self.tree_msg.tick_frequency_hz = 10.0

        self.rate = self.ros_node.create_rate(frequency=self.tree_msg.tick_frequency_hz)
        # Ensure Tree is editable after loading
        with self._state_lock:
            self.tree_msg.state = Tree.EDITABLE

        # find and set root name
        root = self.find_root()
        with self._state_lock:
            if root:
                self.tree_msg.root_name = root.name

        response.success = True
        self.publish_info(self.debug_manager.get_debug_info_msg())
        get_logger("tree_manager").info("Successfully loaded tree")
        if self.publish_diagnostic is None:
            self.set_diagnostics_name()
        return response

    def set_execution_mode(
        self,
        request: SetExecutionMode.Request,
        response: SetExecutionMode.Response,
    ) -> SetExecutionMode.Response:
        """
        Set the parameters of our :class:`DebugManager`.

        :param  ros_bt_msgs.srv.SetExecutionModeRequest request:
        """
        self.debug_manager.set_execution_mode(
            single_step=request.single_step,
            collect_performance_data=request.collect_performance_data,
            publish_subtrees=request.publish_subtrees,
            collect_node_diagnostics=request.collect_node_diagnostics,
        )
        if request.publish_subtrees:
            self.control_execution(
                ControlTreeExecution.Request(
                    command=ControlTreeExecution.Request.SETUP_AND_SHUTDOWN
                ),
                ControlTreeExecution.Response(),
            )
        else:
            self.debug_manager.clear_subtrees()
            self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    def debug_step(
        self, request: Continue.Request, response: Continue.Response
    ) -> Continue.Response:
        """
        Continue execution.

        If single step mode is enabled, advance a single step. If we're
        currently stopped at a breakpoint, continue to the next, if
        any. If we're not stopped at all, do nothing.

        :param request ros_bt_msgs.srv.ContinueRequest:
        """
        # TODO(nberg): Add more logic to at least warn if we're not stopped
        self.debug_manager.continue_debug()
        return Continue.Response(success=True)

    def modify_breakpoints(
        self, request: ModifyBreakpoints.Request, response: ModifyBreakpoints.Response
    ) -> ModifyBreakpoints.Response:
        response.current_breakpoints = self.debug_manager.modify_breakpoints(
            add=request.add, remove=request.remove, remove_all=request.remove_all
        )
        return response

    def control_execution(  # noqa: C901 # TODO: Remove this and simplfy the method.
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
            is_idle = self.get_state() == Tree.IDLE
            if is_idle and self._tick_thread.is_alive():
                self._tick_thread.join(0.5)
                if self._tick_thread.is_alive():
                    raise BehaviorTreeException(
                        "Tried to join tick thread with Tree state " "IDLE, but failed!"
                    )

        # Make a new tick thread if there isn't one or the old one has been
        # successfully joined.
        if self._tick_thread is None or not self._tick_thread.is_alive():
            self._tick_thread = Thread(target=self.tick_report_exceptions)

        tree_state = self.get_state()

        # Check for error state and abort if command is not SHUTDOWN -
        # if it is, we fall through to the if below and shut down the
        # tree
        if (
            tree_state == Tree.ERROR
            and request.command != ControlTreeExecution.Request.SHUTDOWN
        ):
            response.error_message = (
                "Tree is in error state, the only allowed action is SHUTDOWN"
            )
            return response

        if request.command == ControlTreeExecution.Request.SETUP_AND_SHUTDOWN:
            if self._tick_thread.is_alive() or tree_state == Tree.TICKING:
                response.success = False
                response.error_message = (
                    "Tried to setup tree while it is running, aborting"
                )
                response.tree_state = tree_state
                get_logger("tree_manager").warn(response.error_message)
                return response

            self.debug_manager.clear_subtrees()
            try:
                root = self.find_root()
                if root:
                    with self._state_lock:
                        self._setting_up = True
                    root.setup()
                    with self._state_lock:
                        self._setting_up = False
            except TreeTopologyError as ex:
                response.success = False
                response.error_message = str(ex)
                response.tree_state = self.get_state()
                return response
            except BehaviorTreeException as ex:
                response.success = False
                response.error_message = str(ex)
                response.tree_state = self.get_state()
                return response
            response.tree_state = tree_state
            # shutdown the tree after the setup and shutdown request
            request.command = ControlTreeExecution.Request.SHUTDOWN

        if request.command in [
            ControlTreeExecution.Request.STOP,
            ControlTreeExecution.Request.SHUTDOWN,
        ]:
            if tree_state == Tree.TICKING:
                with self._state_lock:
                    self.tree_msg.state = Tree.STOP_REQUESTED
                # Four times the allowed period should be plenty of time to
                # finish the current tick, if the tree has not stopped by then
                # we're in deep trouble.
                if self._tick_thread.is_alive():
                    # Give the tick thread some time to finish
                    self._tick_thread.join(
                        (1.0 / self.tree_msg.tick_frequency_hz) * 4.0
                    )

                    # If we're debugging or setting up (and ROS is not
                    # shutting down), keep sleeping until the thread
                    # finishes
                    while self._tick_thread.is_alive() and rclpy.ok():
                        setting_up = False
                        with self._state_lock:
                            setting_up = self._setting_up
                        if not self.debug_manager.is_debugging() and not setting_up:
                            break
                        self._tick_thread.join(
                            (1.0 / self.tree_msg.tick_frequency_hz) * 4.0
                        )
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException(
                            "Tried to join tick thread after requesting "
                            "stop, but failed!"
                        )
                state_after_joining = self.get_state()
                if state_after_joining == Tree.IDLE:
                    response.tree_state = Tree.IDLE
                    response.success = True
                elif state_after_joining == Tree.ERROR:
                    response.error_message = (
                        f"Error stopping tick: {str(self._last_error)}"
                    )
                    response.success = False
                    get_logger("tree_manager").error(response.error_message)
                else:
                    response.error_message = (
                        f"Successfully stopped ticking, but tree state is "
                        f"{state_after_joining}, not IDLE"
                    )
                    response.success = False
                    get_logger("tree_manager").error(response.error_message)
            elif tree_state == Tree.WAITING_FOR_TICK:
                try:
                    root = self.find_root()
                    if root:
                        root.untick()
                        state = root.state
                        if state in [NodeMsg.IDLE, NodeMsg.PAUSED]:
                            response.tree_state = Tree.IDLE
                            response.success = True
                        else:
                            response.tree_state = Tree.ERROR
                            response.success = False
                            get_logger("tree_manager").error(
                                f"Root node ({str(root)}) state after unticking is neither "
                                f"IDLE nor PAUSED, but {state}"
                            )
                            response.error_message = "Failed to untick root node."
                    else:
                        get_logger("tree_manager").info(
                            "Unticking a tree with no nodes."
                        )
                        response.tree_state = Tree.IDLE
                        response.success = True
                except TreeTopologyError as ex:
                    response.success = False
                    response.error_message = str(ex)

                self.publish_info(self.debug_manager.get_debug_info_msg())
            else:
                get_logger("tree_manager").info(
                    "Received stop command, but tree was not running"
                )
                response.success = True
                response.tree_state = tree_state

            # actually shut down the tree
            if request.command == ControlTreeExecution.Request.SHUTDOWN:
                try:
                    root = self.find_root()
                    if root:
                        root.shutdown()
                    else:
                        get_logger("tree_manager").info(
                            "Shutting down a tree with no nodes."
                        )
                except TreeTopologyError as ex:
                    response.success = False
                    response.error_message = str(ex)

                # Now the tree is editable again - all nodes are in a state
                # where they must be initialized.
                with self._state_lock:
                    self.tree_msg.state = Tree.EDITABLE

            self.publish_info(self.debug_manager.get_debug_info_msg())

        elif request.command == ControlTreeExecution.Request.TICK_ONCE:
            if self._tick_thread.is_alive() or self.get_state() == Tree.TICKING:
                response.success = False
                response.error_message = (
                    "Tried to tick when tree is already running, aborting"
                )
                get_logger("tree_manager").warn(response.error_message)
            else:
                try:
                    # If there are no nodes in the tree, no ticking is needed
                    if not self.find_root():
                        response.success = True
                        return response

                    self._once = True
                    self._stop_after_result = False
                    with self._state_lock:
                        self.tree_msg.state = Tree.TICKING
                    self._tick_thread.start()
                    # Give the tick thread some time to finish
                    self._tick_thread.join(
                        (1.0 / self.tree_msg.tick_frequency_hz) * 4.0
                    )
                    # If we're debugging or setting up (and ROS is not
                    # shutting down), keep sleepin until the thread
                    # finishes
                    while self._tick_thread.is_alive() and rclpy.ok():
                        setting_up = False
                        with self._state_lock:
                            setting_up = self._setting_up
                        if not self.debug_manager.is_debugging() and not setting_up:
                            break
                        self._tick_thread.join(
                            (1.0 / self.tree_msg.tick_frequency_hz) * 4.0
                        )
                    if self._tick_thread.is_alive():
                        raise BehaviorTreeException(
                            "Tried to join tick thread after single "
                            "tick, but failed!"
                        )
                    state_after_joining = self.get_state()
                    if state_after_joining == Tree.WAITING_FOR_TICK:
                        response.tree_state = Tree.WAITING_FOR_TICK
                        response.success = True
                    elif state_after_joining == Tree.ERROR:
                        response.error_message = (
                            f"Error during single tick: {str(self._last_error)}"
                        )
                        response.success = False
                        get_logger("tree_manager").error(response.error_message)
                    else:
                        response.error_message = (
                            f"Successfully stopped ticking, but tree state "
                            f"is {state_after_joining}, not IDLE"
                        )
                        response.success = False
                        get_logger("tree_manager").error(response.error_message)
                except TreeTopologyError as ex:
                    response.success = False
                    response.error_message = str(ex)
                    response.tree_state = self.get_state()

        elif request.command in [
            ControlTreeExecution.Request.TICK_PERIODICALLY,
            ControlTreeExecution.Request.TICK_UNTIL_RESULT,
        ]:
            if self._tick_thread.is_alive() or tree_state == Tree.TICKING:
                response.success = False
                response.error_message = (
                    "Tried to start periodic ticking when tree is "
                    "already running, aborting"
                )
                response.tree_state = tree_state
                get_logger("tree_manager").warn(response.error_message)
            else:
                try:
                    # If there are no nodes in the tree, no ticking is needed
                    if not self.find_root():
                        response.success = True
                        return response

                    with self._state_lock:
                        self.tree_msg.state = Tree.TICKING
                    self._once = False
                    self._stop_after_result = False
                    if (
                        request.command
                        == ControlTreeExecution.Request.TICK_UNTIL_RESULT
                    ):
                        self._stop_after_result = True
                    # Use provided tick frequency, if any
                    if request.tick_frequency_hz != 0:
                        self.tree_msg.tick_frequency_hz = request.tick_frequency_hz
                        self.rate = self.ros_node.create_rate(
                            frequency=self.tree_msg.tick_frequency_hz
                        )
                    if self.tree_msg.tick_frequency_hz == 0:
                        get_logger("tree_manager").warn(
                            "Loaded tree had frequency 0Hz. Defaulting to 10Hz"
                        )
                        self.tree_msg.tick_frequency_hz = 10.0
                        self.rate = self.ros_node.create_rate(
                            frequency=self.tree_msg.tick_frequency_hz
                        )
                    self._tick_thread.start()
                    response.success = True
                    response.tree_state = Tree.TICKING
                except TreeTopologyError as ex:
                    response.success = False
                    response.error_message = str(ex)
                    response.tree_state = self.get_state()

        elif request.command == ControlTreeExecution.Request.RESET:
            if self._tick_thread.is_alive() or tree_state == Tree.TICKING:
                response.success = False
                response.error_message = (
                    "Tried to reset tree while it is running, aborting"
                )
                response.tree_state = tree_state
                get_logger("tree_manager").warn(response.error_message)
            else:
                try:
                    root = self.find_root()
                    if root:
                        root.reset()
                    else:
                        get_logger("tree_manager").info(
                            "Resetting a tree with no root."
                        )
                    with self._state_lock:
                        self.tree_msg.state = Tree.IDLE
                    response.success = True
                    response.tree_state = self.get_state()
                except TreeTopologyError as ex:
                    response.success = False
                    response.error_message = str(ex)
                    response.tree_state = self.get_state()
                except BehaviorTreeException as ex:
                    response.success = False
                    response.error_message = str(ex)
                    response.tree_state = self.get_state()

            self.publish_info(self.debug_manager.get_debug_info_msg())
        elif request.command == ControlTreeExecution.Request.DO_NOTHING:
            get_logger("tree_manager").info("Doing nothing in this request")
            response.success = True
        else:
            response.error_message = f"Received unknown command {request.command}"
            get_logger("tree_manager").error(response.error_message)
            response.success = False

        return response

    @is_edit_service
    def set_simulate_tick(
        self,
        request: SetSimulateTick.Request,
        response: SetSimulateTick.Response,
    ) -> SetSimulateTick.Response:
        """
        Set the simulate tick status to allows to simulate ticks without causing actual actions.

        :param request: The request to process containing the new values.
        :return: The result of the request.
        """
        self.simulate_tick = request.simulate_tick
        self.succeed_always = request.succeed_always

        response.success = True
        return response

    @is_edit_service
    def add_node(
        self, request: AddNode.Request, response: AddNode.Response
    ) -> AddNode.Response:
        """
        Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeRequest request:
          A request describing the node to add.
        """
        internal_request = AddNodeAtIndex.Request(
            parent_name=request.parent_name,
            node=request.node,
            allow_rename=request.allow_rename,
            new_child_index=-1,
        )
        internal_response = AddNodeAtIndex.Response()
        internal_response = self.add_node_at_index(
            request=internal_request, response=internal_response
        )

        response.success = internal_response.success
        response.error_message = internal_response.error_message
        response.actual_node_name = internal_response.actual_node_name
        return response

    @is_edit_service
    def add_node_at_index(
        self, request: AddNodeAtIndex.Request, response: AddNodeAtIndex.Response
    ) -> AddNodeAtIndex.Response:
        """
        Add the node in this request to the tree.

        :param ros_bt_py_msgs.srv.AddNodeAtIndexRequest request:
            A request describing the node to add.
        """
        try:
            instance = self.instantiate_node_from_msg(
                request.node, request.allow_rename
            )
            response.success = True
            response.actual_node_name = instance.name
        except BehaviorTreeException as exc:
            response.success = False
            response.error_message = str(exc)
            return response

        instance.succeed_always = self.succeed_always
        instance.simulate_tick = self.simulate_tick

        # Add node as child of the named parent, if any
        if request.parent_name:
            if request.parent_name not in self.nodes:
                response.success = False
                response.error_message = (
                    f"Parent {request.parent_name} of node "
                    f"{instance.name} does not exist!"
                )
                # Remove node from tree
                self.remove_node(
                    RemoveNode.Request(node_name=instance.name, remove_children=False)
                )
                return response
            self.nodes[request.parent_name].add_child(
                child=instance, at_index=request.new_child_index
            )

        # Add children from msg to node
        missing_children = []
        for child_name in request.node.child_names:
            if child_name in self.nodes:
                instance.add_child(self.nodes[child_name])
            else:
                missing_children.append(child_name)
        if missing_children:
            response.success = False
            response.error_message = (
                f"Children for node {instance.name} are not "
                f"in tree: {str(missing_children)}"
            )
            # Remove node from tree to restore state before insertion attempt
            self.remove_node(
                RemoveNode.Request(node_name=instance.name, remove_children=False)
            )

        nodes_in_cycles = self.find_nodes_in_cycles()
        if nodes_in_cycles:
            response.success = False
            response.error_message = (
                f"Found cycles in tree {self.tree_msg.name} after inserting node "
                f"{request.node.name} as {response.actual_node_name}. "
                f"Nodes in cycles: {str(nodes_in_cycles)}"
            )
            # First, remove all of the node's children to avoid infinite
            # recursion in remove_node()
            for child_name in [c.name for c in instance.children]:
                instance.remove_child(child_name)

            # Then remove the node from the tree
            self.remove_node(
                RemoveNode.Request(node_name=instance.name, remove_children=False)
            )
            return response
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
    def reload_tree(
        self, request: Optional[ReloadTree.Request], response: ReloadTree.Response
    ) -> ReloadTree.Response:
        """Reload the currently loaded tree."""
        load_response = LoadTree.Response()
        load_response = self.load_tree(
            request=LoadTree.Request(tree=self.tree_msg), response=load_response
        )

        response.success = load_response.success
        response.error_message = load_response.error_message

        return response

    @is_edit_service
    def change_tree_name(
        self, request: ChangeTreeName.Request, response: ChangeTreeName.Response
    ) -> ChangeTreeName.Response:
        """Change the name of the currently loaded tree."""
        self.tree_msg.name = request.name

        self.publish_info(self.debug_manager.get_debug_info_msg())

        response.success = True

        return response

    @is_edit_service
    def remove_node(
        self, request: RemoveNode.Request, response: RemoveNode.Response
    ) -> RemoveNode.Response:
        """
        Remove the node identified by `request.node_name` from the tree.

        If the parent of the node removed supports enough children to
        take on all of the removed node's children, it will. Otherwise,
        children will be orphaned.
        """
        if request.node_name not in self.nodes:
            response.success = False
            response.error_message = (
                f"No node with name {request.node_name} in tree {self.tree_msg.name}"
            )
            return response

        # Shutdown node - this should also shutdown all children, but you
        # never know, so check later.
        self.nodes[request.node_name].shutdown()
        names_to_remove = [request.node_name]
        if request.remove_children:
            add_children_of = [request.node_name]
            children_added = set()
            while add_children_of:
                name = add_children_of.pop()
                if name not in self.nodes:
                    response.success = False
                    response.error_message = (
                        f"Error while removing children of node {request.node_name}: "
                        f"No node with name {name} in tree {self.tree_msg.name}"
                    )
                    return response
                if name not in children_added:
                    names_to_remove.extend(
                        [child.name for child in self.nodes[name].children]
                    )
                    add_children_of.extend(
                        [child.name for child in self.nodes[name].children]
                    )
        else:
            # If we're not removing the children, at least set their parent to None
            for child in self.nodes[request.node_name].children:
                child.parent = None
        # Remove nodes in the reverse order they were added to the
        # list, i.e. the "deepest" ones first. This ensures that the
        # parent we refer to in the error message still exists.

        # set to prevent us from removing a node twice - no node
        # *should* have more than one parent, but better safe than
        # sorry
        removed_names = set()
        for name in reversed(names_to_remove):
            if name in removed_names:
                continue
            removed_names.add(name)
            # Check if node is already in shutdown state. If not, call
            # shutdown, but warn, because the parent node should have
            # done that!
            if self.nodes[name].state != NodeMsg.SHUTDOWN:
                # It's reasonable to expect parent to not be None here, since
                # the node is one of a list of children
                parent_name = self.nodes[name].parent.name
                get_logger("tree_manager").warn(
                    f"Node {name} was not shut down. Check parent node {parent_name} "
                    f"({type(self.nodes[parent_name]).__name__}) "
                    f"for proper implementation of _do_shutdown()",
                )
                self.nodes[name].shutdown()

            # If we have a parent, remove the node from that parent
            if self.nodes[name].parent and self.nodes[name].parent.name in self.nodes:
                self.nodes[self.nodes[name].parent.name].remove_child(name)
            del self.nodes[name]

        # Unwire wirings that have removed nodes as source or target
        self.unwire_data(
            WireNodeData.Request(
                wirings=[
                    wiring
                    for wiring in self.tree_msg.data_wirings
                    if (
                        wiring.source.node_name in names_to_remove
                        or wiring.target.node_name in names_to_remove
                    )
                ]
            )
        )

        # Keep tree_msg up-to-date
        self.tree_msg.data_wirings = [
            wiring
            for wiring in self.tree_msg.data_wirings
            if (
                wiring.source.node_name not in names_to_remove
                and wiring.target.node_name not in names_to_remove
            )
        ]
        self.tree_msg.public_node_data = [
            data
            for data in self.tree_msg.public_node_data
            if data.node_name not in names_to_remove
        ]

        response.success = True
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
    def morph_node(
        self, request: MorphNode.Request, response: MorphNode.Response
    ) -> MorphNode.Response:
        """Morphs the flow control node into the new node provided in `request.new_node`."""
        if request.node_name not in self.nodes:
            response.success = False
            response.error_message = (
                f"No node with name {request.node_name} in tree {self.tree_msg.name}"
            )
            return response

        old_node = self.nodes[request.node_name]

        try:
            new_node = Node.from_msg(request.new_node)
        except (TypeError, BehaviorTreeException) as exc:
            response.success = False
            response.error_message = f"Error instantiating node {str(exc)}"
            return response

        # First unwire all data connection to the existing node
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self.tree_msg.data_wirings
                if old_node.name in [wiring.source.node_name, wiring.target.node_name]
            ]
        )

        unwire_resp = self.unwire_data(wire_request)
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
            old_child_index = parent.get_child_index(old_node.name)

            if old_child_index is None:
                return MorphNode.Response(
                    success=False,
                    error_message=(
                        f"Parent of node {old_node.name} claims to have no child with that name?!"
                    ),
                )
            parent.remove_child(old_node.name)

            try:
                parent.add_child(new_node, at_index=old_child_index)
            except (KeyError, BehaviorTreeException) as ex:
                error_message = (
                    f"Failed to add new instance of node {old_node.name}: {str(ex)}"
                )
                try:
                    parent.add_child(old_node, at_index=old_child_index)
                    rewire_resp = self.wire_data(wire_request)
                    if not get_success(rewire_resp):
                        error_message += (
                            f"\nAlso failed to restore data wirings: "
                            f"{get_error_message(rewire_resp)}"
                        )
                        with self._state_lock:
                            self.tree_msg.state = Tree.ERROR

                except (KeyError, BehaviorTreeException):
                    error_message += "\n Also failed to restore old node."
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR

                return MorphNode.Response(success=False, error_message=error_message)

        # Move the children from old to new
        for child_name in [child.name for child in old_node.children]:
            child = old_node.remove_child(child_name)
            if child_name != new_node.name:
                new_node.add_child(child)

        # Add the new node to self.nodes
        del self.nodes[old_node.name]
        self.nodes[new_node.name] = new_node

        # Re-wire all the data, just as it was before
        # FIXME: this should be a best-effort rewiring, only re-wire identical input/outputs
        new_wire_request = deepcopy(wire_request)

        rewire_resp = self.wire_data(new_wire_request)
        if not get_success(rewire_resp):
            response.error_message = (
                f"Failed to re-wire data to new node {new_node.name}:"
                f" {get_error_message(rewire_resp)}"
            )
            response.success = False
            return response

        response.success = True
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
    def set_options(  # noqa: C901
        self, request: SetOptions.Request, response: SetOptions.Response
    ) -> SetOptions.Response:
        """
        Set the option values of a given node.

        This is an "edit service", i.e. it can only be used when the
        tree has not yet been initialized or has been shut down.
        """
        if request.node_name not in self.nodes:
            return SetOptions.Response(
                success=False,
                error_message=(
                    f"Unable to find node {request.node_name} in tree "
                    f"{self.tree_msg.name}"
                ),
            )

        if (
            request.rename_node
            and request.new_name != request.node_name
            and request.new_name in self.nodes
        ):
            return SetOptions.Response(
                success=False,
                error_message=(
                    f"Unable to rename node {request.node_name} to {request.new_name} "
                    "- a node with that name exists already."
                ),
            )

        node = self.nodes[request.node_name]
        unknown_options = []
        preliminary_incompatible_options = []
        try:
            deserialized_options = dict(
                (option.key, json_decode(option.serialized_value))
                for option in request.options
            )
        except ValueError as ex:
            return SetOptions.Response(
                success=False,
                error_message=f"Failed to deserialize option value: {str(ex)}",
            )
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
                        message_converter.convert_dictionary_to_ros_message(
                            other_type, deserialized_options[key]
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
            return SetOptions.Response(
                success=False, error_message="\n".join(error_strings)
            )

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
        new_node = node.__class__(
            options=deserialized_options,
            name=request.new_name if request.rename_node else node.name,
            debug_manager=node.debug_manager,
        )

        # Use this request to unwire any data connections the existing
        # node has - if we didn't do this, the node wouldn't ever be
        # garbage collected, among other problems.
        #
        # We'll use the same request to re-wire the connections to the
        # new node (or the old one, if anything goes wrong).
        wire_request = WireNodeData.Request(
            wirings=[
                wiring
                for wiring in self.tree_msg.data_wirings
                if node.name in [wiring.source.node_name, wiring.target.node_name]
            ]
        )

        unwire_resp = self.unwire_data(wire_request)
        if not get_success(unwire_resp):
            return SetOptions.Response(
                success=False,
                error_message=(
                    f"Failed to unwire data for node {node.name}: "
                    f"{get_error_message(unwire_resp)}"
                ),
            )

        parent = None
        if node.parent:
            parent = node.parent
            # Remember the old index so we can insert the new instance at
            # the same position
            old_child_index = parent.get_child_index(node.name)

            if old_child_index is None:
                return SetOptions.Response(
                    success=False,
                    error_message=(
                        f"Parent of node {node.name} claims to "
                        f"have no child with that name?!"
                    ),
                )

            try:
                parent.remove_child(node.name)
            except KeyError as ex:
                error_message = (
                    f"Failed to remove old instance of node {node.name}: {str(ex)}"
                )
                rewire_resp = self.wire_data(wire_request)
                if not get_success(rewire_resp):
                    error_message += (
                        "\nAlso failed to restore data wirings: "
                        f"{get_error_message(rewire_resp)}"
                    )
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR

                return SetOptions.Response(success=False, error_message=error_message)

            try:
                parent.add_child(new_node, at_index=old_child_index)
            except (KeyError, BehaviorTreeException) as ex:
                error_message = (
                    f"Failed to add new instance of node {node.name}: {str(ex)}"
                )
                try:
                    parent.add_child(node, at_index=old_child_index)
                    rewire_resp = self.wire_data(wire_request)
                    if not get_success(rewire_resp):
                        error_message += (
                            f"\nAlso failed to restore data wirings: "
                            f"{get_error_message(rewire_resp)}"
                        )
                        with self._state_lock:
                            self.tree_msg.state = Tree.ERROR

                except (KeyError, BehaviorTreeException):
                    error_message += "\n Also failed to restore old node."
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR

                return SetOptions.Response(success=False, error_message=error_message)

        # Add the new node to self.nodes
        del self.nodes[node.name]
        self.nodes[new_node.name] = new_node

        # Re-wire all the data, just as it was before
        new_wire_request = deepcopy(wire_request)
        if request.rename_node:
            for wiring in new_wire_request.wirings:
                if wiring.source.node_name == node.name:
                    wiring.source.node_name = new_node.name
                if wiring.target.node_name == node.name:
                    wiring.target.node_name = new_node.name

        rewire_resp = self.wire_data(new_wire_request)
        if not get_success(rewire_resp):
            error_message = (
                f"Failed to re-wire data to new node {new_node.name}: "
                f"{get_error_message(rewire_resp)}"
            )
            # Try to undo everything, starting with removing the new
            # node from the node dict
            del self.nodes[new_node.name]
            self.nodes[node.name] = node

            if parent is not None:
                try:
                    parent.remove_child(new_node.name)
                    parent.add_child(node, at_index=old_child_index)
                except (KeyError, BehaviorTreeException) as ex:
                    error_message += f"\nError restoring old node: {str(ex)}"

            # Now try to re-do the wirings
            recovery_wire_response = self.wire_data(wire_request)
            if not get_success(recovery_wire_response):
                error_message += (
                    f"\nFailed to re-wire data to restored node {node.name}: "
                    f"{get_error_message(recovery_wire_response)}"
                )

                with self._state_lock:
                    self.tree_msg.state = Tree.ERROR
            return SetOptions.Response(success=False, error_message=error_message)

        # Move all of node's children to new_node
        try:
            # This line is important: The list comprehension creates a
            # new list that won't be affected by calling
            # remove_child()!
            for child_name in [child.name for child in node.children]:
                get_logger("tree_manager").info(f"Moving child {child_name}")
                new_node.add_child(node.remove_child(child_name))
        except BehaviorTreeException as exc:
            with self._state_lock:
                self.tree_msg.state = Tree.ERROR

            return SetOptions.Response(
                success=False,
                error_message=f"Failed to transfer children to new node: {str(exc)}",
            )

        # We made it!
        self.publish_info(self.debug_manager.get_debug_info_msg())
        return SetOptions.Response(success=True)

    @is_edit_service
    def move_node(
        self, request: MoveNode.Request, response: MoveNode.Response
    ) -> MoveNode.Response:
        """Move the named node to a different parent and insert it at the given index."""
        if request.node_name not in self.nodes:
            return MoveNode.Response(
                success=False,
                error_message=f'Node to be moved ("{request.node_name}") is not in tree.',
            )

        # Empty parent name -> just remove node from parent
        if request.new_parent_name == "":
            node = self.nodes[request.node_name]
            if node.parent is not None:
                node.parent.remove_child(node.name)
            self.publish_info(self.debug_manager.get_debug_info_msg())
            return MoveNode.Response(success=True)

        if request.new_parent_name not in self.nodes:
            return MoveNode.Response(
                success=False,
                error_message=f'New parent ("{request.new_parent_name}") is not in tree.',
            )

        new_parent_max_children = self.nodes[
            request.new_parent_name
        ].node_config.max_children
        if (
            new_parent_max_children is not None
            and len(self.nodes[request.new_parent_name].children)
            == new_parent_max_children
        ):
            return MoveNode.Response(
                success=False,
                error_message=(
                    f"Cannot move node {request.node_name} to new parent node "
                    f"{request.new_parent_name}. "
                    "Parent node already has the maximum number "
                    f"of children ({new_parent_max_children})."
                ),
            )

        # If the new parent is part of the moved node's subtree, we'd
        # get a cycle, so check for that and fail if true!
        if request.new_parent_name in [
            subtree_node.name
            for subtree_node in self.nodes[request.node_name].get_subtree_msg()[0].nodes
        ]:
            return MoveNode.Response(
                success=False,
                error_message=(
                    f"Cannot move node {request.node_name} to new parent node "
                    f"{request.new_parent_name}. "
                    f"{request.new_parent_name} is a child of {request.node_name}!"
                ),
            )

        # Remove node from old parent, if any
        old_parent = self.nodes[request.node_name].parent
        if old_parent is not None:
            old_parent.remove_child(request.node_name)

        # Add node to new parent
        self.nodes[request.new_parent_name].add_child(
            child=self.nodes[request.node_name], at_index=request.new_child_index
        )

        self.publish_info(self.debug_manager.get_debug_info_msg())
        return MoveNode.Response(success=True)

    @is_edit_service
    def replace_node(
        self, request: ReplaceNode.Request, response: ReplaceNode.Response
    ) -> ReplaceNode.Response:
        """
        Replace the named node with `new_node`.

        Will also move all children of the old node to the new one, but
        only if `new_node` supports that number of children. Otherwise,
        this will return an error and leave the tree unchanged.
        """
        if request.old_node_name not in self.nodes:
            return ReplaceNode.Response(
                success=False,
                error_message=f'Node to be replaced ("{request.old_node_name}") is not in tree.',
            )
        if request.new_node_name not in self.nodes:
            return ReplaceNode.Response(
                success=False,
                error_message=f'Replacement node ("{request.new_node_name}") is not in tree.',
            )

        old_node = self.nodes[request.old_node_name]

        new_node_max_children = self.nodes[
            request.new_node_name
        ].node_config.max_children
        # We're *replacing* one of the children, so there should only
        # be an issue if there were too many children before. Which
        # shouldn't happen. But you know, better safe than sorry!
        if (
            new_node_max_children is not None
            and len(old_node.children) > new_node_max_children
        ):
            return ReplaceNode.Response(
                success=False,
                error_message=(
                    f'Replacement node ("{request.new_node_name}") does not support the number of'
                    f"children required ({request.old_node_name} has "
                    f"{len(old_node.children)} children, "
                    f"{request.new_node_name} supports {new_node_max_children}."
                ),
            )

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
                if child.name == request.old_node_name:
                    old_node_child_index = index
                    break

        # Get the new node
        new_node = self.nodes[request.new_node_name]

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
            and new_node.parent.name == old_node_parent.name
            and old_node_child_index > 0
        ):
            for index, child in enumerate(new_node.parent.children):
                if child.name == request.new_node_name:
                    if index < old_node_child_index:
                        old_node_child_index -= 1
                    break

        # Move the children from old to new
        for child_name in [child.name for child in old_node.children]:
            child = old_node.remove_child(child_name)
            if child_name != new_node.name:
                new_node.add_child(child)
        # Remove the old node (we just moved the children, so we can
        # set remove_children to True)
        res = RemoveNode.Response()
        res = self.remove_node(
            RemoveNode.Request(node_name=request.old_node_name, remove_children=True),
            res,
        )

        if not get_success(res):
            with self._state_lock:
                self.tree_msg.state = Tree.ERROR
            self.publish_info(self.debug_manager.get_debug_info_msg())
            return ReplaceNode.Response(
                success=False,
                error_message=f'Could not remove old node: "{get_error_message(res)}"',
            )

        # Move the new node to the old node's parent (if it had one)
        if old_node_parent is not None:
            self.move_node(
                MoveNode.Request(
                    node_name=request.new_node_name,
                    new_parent_name=old_node_parent.name,
                    new_child_index=old_node_child_index,
                ),
                MoveNode.Response(),
            )

        self.publish_info(self.debug_manager.get_debug_info_msg())
        return ReplaceNode.Response(success=True)

    @is_edit_service
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
        try:
            root = self.find_root()
            if not root:
                raise TreeTopologyError("No nodes in tree")
        except TreeTopologyError as ex:
            response.success = False
            response.error_message = f"Unable to find root node: {str(ex)}"
            return response

        successful_wirings = []
        for wiring in request.wirings:
            target_node = root.find_node(wiring.target.node_name)
            if not target_node:
                response.success = False
                response.error_message = (
                    f"Target node {wiring.target.node_name} does not exist"
                )
                break
            try:
                target_node.wire_data(wiring)
                successful_wirings.append(wiring)
            except (KeyError, BehaviorTreeException) as ex:
                if not request.ignore_failure:
                    response.success = False
                    response.error_message = (
                        f'Failed to execute wiring "{wiring}": {str(ex)}'
                    )
                    break

        if not response.success:
            # Undo the successful wirings
            for wiring in successful_wirings:
                target_node = root.find_node(wiring.target.node_name)
                try:
                    target_node.unwire_data(wiring)
                except (KeyError, BehaviorTreeException) as ex:
                    response.success = False
                    response.error_message = (
                        f'Failed to undo wiring "{wiring}": {str(ex)}\n'
                        f"Previous error: {response.error_message}"
                    )
                    get_logger("tree_manager").error(
                        "Failed to undo successful wiring after error. "
                        "Tree is in undefined state!"
                    )
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR
                    break

        # only actually wire any data if there were no errors
        if response.success:
            # We made it here, so all the Wirings should be valid. Time to save
            # them.
            self.tree_msg.data_wirings.extend(successful_wirings)

            self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    @is_edit_service
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
        response.success = True
        try:
            root = self.find_root()
            if not root:
                raise TreeTopologyError("No nodes in tree")
        except TreeTopologyError as ex:
            response.success = False
            response.error_message = f"Unable to find root node: {str(ex)}"
            return response

        successful_unwirings = []
        for wiring in request.wirings:
            target_node = root.find_node(wiring.target.node_name)
            if not target_node:
                response.success = False
                response.error_message = (
                    f"Target node {wiring.target.node_name} does not exist"
                )
                break
            try:
                target_node.unwire_data(wiring)
                successful_unwirings.append(wiring)
            except (KeyError, BehaviorTreeException) as ex:
                response.success = False
                response.error_message = (
                    f'Failed to remove wiring "{wiring}": {str(ex)}'
                )
                break

        if not response.success:
            # Re-Wire the successful unwirings
            for wiring in successful_unwirings:
                target_node = root.find_node(wiring.target.node_name)
                try:
                    target_node.wire_data(wiring)
                except (KeyError, BehaviorTreeException) as ex:
                    response.success = False
                    response.error_message = (
                        f'Failed to redo wiring "{wiring}": {str(ex)}\n'
                        f"Previous error: {response.error_message}"
                    )
                    get_logger("tree_manager").error(
                        "Failed to rewire successful unwiring after error. "
                        "Tree is in undefined state!"
                    )
                    with self._state_lock:
                        self.tree_msg.state = Tree.ERROR
                    break

        # only actually wire any data if there were no errors
        if response.success:
            # We've removed these NodeDataWirings, so remove them from tree_msg as
            # well.
            for wiring in request.wirings:
                if wiring in self.tree_msg.data_wirings:
                    self.tree_msg.data_wirings.remove(wiring)

            self.publish_info(self.debug_manager.get_debug_info_msg())
        return response

    def get_subtree(self, request: GetSubtree.Request) -> GetSubtree.Response:
        if request.subtree_root_name not in self.nodes:
            return GetSubtree.Response(
                success=False,
                error_message=f'Node "{request.subtree_root_name}" does not exist!',
            )
        try:
            return GetSubtree.Response(
                success=True,
                subtree=self.nodes[request.subtree_root_name].get_subtree_msg()[0],
            )
        except BehaviorTreeException as exc:
            return GetSubtree.Response(
                success=False,
                error_message=(
                    f"Error retrieving subtree rooted at {request.subtree_root_name}: {str(exc)}"
                ),
            )

    def generate_subtree(
        self, request: GenerateSubtree.Request, response: GenerateSubtree.Response
    ) -> GenerateSubtree.Response:
        """
        Generate a subtree generated from the provided list of nodes and the loaded tree.

        This also adds all relevant parents to the tree message, resulting in a tree that is
        executable and does not contain any orpahned nodes.
        """
        whole_tree = deepcopy(self.tree_msg)

        root = self.find_root()

        if not root:
            response.success = False
            response.error_message = "No tree message available"
        else:
            nodes = set()
            for node in whole_tree.nodes:
                nodes.add(node.name)

            nodes_to_keep = set()
            nodes_to_remove = set()
            for node in whole_tree.nodes:
                for search_node in request.nodes:
                    if node.name == search_node or search_node in node.child_names:
                        nodes_to_keep.add(node.name)

            for node in nodes:
                if node not in nodes_to_keep:
                    nodes_to_remove.add(node)

            manager = TreeManager(
                ros_node=self.ros_node,
                name="temporary_tree_manager",
                publish_tree_callback=lambda *args: None,
                publish_debug_info_callback=lambda *args: None,
                publish_debug_settings_callback=lambda *args: None,
                debug_manager=DebugManager(),
            )

            load_response = manager.load_tree(
                request=LoadTree.Request(tree=whole_tree), prefix=""
            )

            if load_response.success:
                for node_name in nodes_to_remove:
                    manager.remove_node(
                        RemoveNode.Request(node_name=node_name, remove_children=False)
                    )
                root = manager.find_root()
                if not root:
                    get_logger("tree_manager").info("No nodes in tree")
                else:
                    manager.tree_msg.root_name = root.name
                response.success = True
                response.tree = manager.to_msg()
        return response

    #########################
    # Service Handlers Done #
    #########################

    def instantiate_node_from_msg(self, node_msg, allow_rename, permissive=False):
        try:
            node_instance = Node.from_msg(
                node_msg, debug_manager=self.debug_manager, permissive=permissive
            )
        except TypeError as exc:
            raise BehaviorTreeException(str(exc))
        except AttributeError as exc:
            raise BehaviorTreeException(str(exc))

        if node_instance.name in self.nodes:
            if allow_rename:
                node_instance.name = self.make_name_unique(node_instance.name)
            else:
                raise BehaviorTreeException(
                    f'Node with name "{node_instance.name}" exists already'
                )

        self.nodes[node_instance.name] = node_instance

        return node_instance

    def make_name_unique(self, name):
        while name in self.nodes:
            name = increment_name(name)
        return name

    def to_msg(self, ticked=False):
        if ticked:
            # early exit during ticking
            self.tree_msg.nodes = [node.to_msg() for node in self.nodes.values()]
            return self.tree_msg
        try:
            root = self.find_root()
            if root is not None:
                subtree = root.get_subtree_msg()[0]
                self.tree_msg.nodes = subtree.nodes
                self.tree_msg.public_node_data = subtree.public_node_data
            else:
                self.tree_msg.nodes = []
        except TreeTopologyError as exc:
            get_logger("tree_manager").warn(f"Strange topology {exc}")
            # build a tree_msg out of this strange topology, so the user can fix it in the editor
            self.tree_msg.nodes = [node.to_msg() for node in self.nodes.values()]
        return self.tree_msg


def get_success(response):
    if isinstance(response, dict):
        return response["success"]

    return response.success


def get_error_message(response):
    if isinstance(response, dict):
        return response["error_message"]

    return response.error_message
