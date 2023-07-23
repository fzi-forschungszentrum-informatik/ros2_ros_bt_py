#!/usr/bin/env python


"""Module containing the main node for a ros_bt_py instance running the BT."""

from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import ParameterDescriptor, ReentrantCallbackGroup, Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray
from ros_bt_py_interfaces.msg import (
    Tree,
    DebugInfo,
    DebugSettings,
    NodeDiagnostics,
    Messages,
    Packages,
)
from ros_bt_py_interfaces.srv import (
    AddNode,
    AddNodeAtIndex,
    ControlTreeExecution,
    ModifyBreakpoints,
    RemoveNode,
    WireNodeData,
    GetAvailableNodes,
    SetExecutionMode,
    SetOptions,
    Continue,
    LoadTree,
    LoadTreeFromPath,
    MoveNode,
    ReplaceNode,
    GetSubtree,
    ClearTree,
    MorphNode,
    SaveTree,
    FixYaml,
    GetMessageFields,
    GetPackageStructure,
    GenerateSubtree,
    ReloadTree,
    ChangeTreeName,
)

from ros_bt_py.tree_manager import (
    TreeManager,
    get_success,
    get_error_message,
    get_available_nodes,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.package_manager import PackageManager
from ros_bt_py.helpers import fix_yaml


class TreeNode(Node):
    """ROS node running a single behavior tree."""

    def __init__(self, tree_name: str = "BehaviorTree"):
        """Create a new Behavior Tree Node with all required publishers and subscribers."""
        super().__init__(tree_name)
        self.get_logger().info("initializing tree node...")
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "node_modules",
                    [""],
                    ParameterDescriptor(
                        name="node_modules",
                        type=ParameterType.PARAMETER_STRING_ARRAY,
                        description="Node modules to load on startup!",
                    ),
                ),
                (
                    "show_traceback_on_exception",
                    False,
                    ParameterDescriptor(
                        name="show_traceback_on_exception",
                        type=ParameterType.PARAMETER_BOOL,
                        description="Show error traceback on exception!",
                    ),
                ),
                (
                    "load_default_tree",
                    False,
                    ParameterDescriptor(
                        name="load_default_tree",
                        type=ParameterType.PARAMETER_BOOL,
                        description="Allow permissive loading of default BT",
                    ),
                ),
                (
                    "load_default_tree_permissive",
                    False,
                    ParameterDescriptor(
                        name="load_default_tree_permissive",
                        type=ParameterType.PARAMETER_BOOL,
                        description="Allow permissive loading of default BT",
                    ),
                ),
                (
                    "default_tree_tick_frequency_hz",
                    10,
                    ParameterDescriptor(
                        name="default_tree_tick_frequency_hz",
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Tick frequency for the default tree!",
                    ),
                ),
                (
                    "default_tree_diagnostics_frequency_hz",
                    1,
                    ParameterDescriptor(
                        name="default_tree_diagnostics_frequency_hz",
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Default frequency to publish diagnostics updates!",
                    ),
                ),
                (
                    "default_tree_control_command",
                    2,
                    ParameterDescriptor(
                        name="default_tree_control_command",
                        type=ParameterType.PARAMETER_INTEGER,
                        description="Default command when running the tree!",
                    ),
                ),
                (
                    "default_tree_path",
                    "",
                    ParameterDescriptor(
                        name="default_tree_path",
                        type=ParameterType.PARAMETER_STRING,
                        description="Path to the behavior tree to load on startup!",
                    ),
                ),
            ],
        )
        node_module_names = self.get_parameter("node_modules").value
        if isinstance(node_module_names, str):
            if "," in node_module_names:
                # Try to parse comma-separated list of modules
                node_module_names = [
                    module.strip() for module in node_module_names.split(",")
                ]
            else:
                # try to parse whitespace-separated list of modules
                node_module_names = node_module_names.split()
        if not isinstance(node_module_names, list):
            raise TypeError(
                f"node_modules must be a list, but is a {type(node_module_names.__name__)}"
            )

        show_traceback_on_exception = bool(
            self.get_parameter("show_traceback_on_exception").value
        )
        load_default_tree = bool(self.get_parameter("load_default_tree").value)
        load_default_tree_permissive = bool(
            self.get_parameter("load_default_tree_permissive").value
        )
        default_tree_path = str(self.get_parameter("default_tree_path").value)
        default_tree_tick_frequency_hz = int(
            self.get_parameter("default_tree_tick_frequency_hz").value
        )
        default_tree_diagnostics_frequency_hz = int(
            self.get_parameter("default_tree_diagnostics_frequency_hz").value
        )
        default_tree_control_command = int(
            self.get_parameter("default_tree_control_command").value
        )

        self.publisher_callback_group = ReentrantCallbackGroup()
        self.tree_pub = self.create_publisher(
            Tree,
            "~/tree",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.debug_info_pub = self.create_publisher(
            DebugInfo,
            "~/debug/debug_info",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.debug_settings_pub = self.create_publisher(
            DebugSettings,
            "~/debug/debug_settings",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.node_diagnostics_pub = self.create_publisher(
            NodeDiagnostics,
            "~/debug/node_diagnostics",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.tick_frequency_pub = self.create_publisher(
            Float64,
            "~/debug/tick_frequency",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.ros_diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.debug_manager = DebugManager(ros_node=self)
        self.tree_manager = TreeManager(
            ros_node=self,
            module_list=node_module_names,
            debug_manager=self.debug_manager,
            publish_tree_callback=self.tree_pub.publish,
            publish_debug_info_callback=self.debug_info_pub.publish,
            publish_debug_settings_callback=self.debug_settings_pub.publish,
            publish_node_diagnostics_callback=self.node_diagnostics_pub.publish,
            publish_diagnostic_callback=self.ros_diagnostics_pub.publish,
            publish_tick_frequency_callback=self.tick_frequency_pub.publish,
            diagnostics_frequency=default_tree_diagnostics_frequency_hz,
            show_traceback_on_exception=show_traceback_on_exception,
        )
        self.service_callback_group = ReentrantCallbackGroup()

        self.add_node_service = self.create_service(
            AddNode,
            "~/add_node",
            callback=self.tree_manager.add_node,
            callback_group=self.service_callback_group,
        )
        self.add_node_at_index_service = self.create_service(
            AddNodeAtIndex,
            "~/add_node_at_index",
            callback=self.tree_manager.add_node_at_index,
            callback_group=self.service_callback_group,
        )
        self.remove_node_service = self.create_service(
            RemoveNode,
            "~/remove_node",
            callback=self.tree_manager.remove_node,
            callback_group=self.service_callback_group,
        )
        self.morph_node_service = self.create_service(
            MorphNode,
            "~/morph_node",
            callback=self.tree_manager.morph_node,
            callback_group=self.service_callback_group,
        )
        self.wire_data_service = self.create_service(
            WireNodeData,
            "~/wire_data",
            callback=self.tree_manager.wire_data,
            callback_group=self.service_callback_group,
        )
        self.unwire_data_service = self.create_service(
            WireNodeData,
            "~/unwire_data",
            callback=self.tree_manager.unwire_data,
            callback_group=self.service_callback_group,
        )
        self.modify_breakpoints_service = self.create_service(
            ModifyBreakpoints,
            "~/debug/modify_breakpoints",
            callback=self.tree_manager.modify_breakpoints,
            callback_group=self.service_callback_group,
        )
        self.control_tree_execution_service = self.create_service(
            ControlTreeExecution,
            "~/control_tree_execution",
            callback=self.tree_manager.control_execution,
            callback_group=self.service_callback_group,
        )
        self.get_available_nodes_service = self.create_service(
            GetAvailableNodes,
            "~/get_available_nodes",
            callback=get_available_nodes,
            callback_group=self.service_callback_group,
        )
        self.get_subtree_service = self.create_service(
            GetSubtree,
            "~/get_subtree",
            callback=self.tree_manager.get_subtree,
            callback_group=self.service_callback_group,
        )
        self.generate_subtree_service = self.create_service(
            GenerateSubtree,
            "~/generate_subtree",
            callback=self.tree_manager.generate_subtree,
            callback_group=self.service_callback_group,
        )
        self.set_execution_mode_service = self.create_service(
            SetExecutionMode,
            "~/debug/set_execution_mode",
            callback=self.tree_manager.set_execution_mode,
            callback_group=self.service_callback_group,
        )
        self.set_options_service = self.create_service(
            SetOptions,
            "~/set_options",
            callback=self.tree_manager.set_options,
            callback_group=self.service_callback_group,
        )
        self.move_node_service = self.create_service(
            MoveNode,
            "~/move_node",
            callback=self.tree_manager.move_node,
            callback_group=self.service_callback_group,
        )
        self.replace_node_service = self.create_service(
            ReplaceNode,
            "~/replace_node",
            callback=self.tree_manager.replace_node,
            callback_group=self.service_callback_group,
        )
        self.continue_service = self.create_service(
            Continue,
            "~/debug/continue",
            callback=self.tree_manager.debug_step,
            callback_group=self.service_callback_group,
        )
        self.load_tree_service = self.create_service(
            LoadTree,
            "~/load_tree",
            callback=self.tree_manager.load_tree,
            callback_group=self.service_callback_group,
        )
        self.load_tree_from_path_service = self.create_service(
            LoadTreeFromPath,
            "~/load_tree_from_path",
            callback=self.tree_manager.load_tree_from_path,
            callback_group=self.service_callback_group,
        )
        self.clear_service = self.create_service(
            ClearTree,
            "~/clear",
            callback=self.tree_manager.clear,
            callback_group=self.service_callback_group,
        )
        self.reload_service = self.create_service(
            ReloadTree,
            "~/reload",
            callback=self.tree_manager.reload_tree,
            callback_group=self.service_callback_group,
        )
        self.change_tree_name_service = self.create_service(
            ChangeTreeName,
            "~/change_tree_name",
            callback=self.tree_manager.change_tree_name,
            callback_group=self.service_callback_group,
        )
        self.fix_yaml_service = self.create_service(
            FixYaml,
            "~/fix_yaml",
            callback=fix_yaml,
            callback_group=self.service_callback_group,
        )

        self.get_logger().info("initialized tree manager")

        if load_default_tree:
            self.get_logger().warn(f"loading default tree: {default_tree_path}")
            tree = Tree(path=default_tree_path)
            load_tree_request = LoadTree.Request(
                tree=tree, permissive=load_default_tree_permissive
            )
            load_tree_response = self.tree_manager.load_tree(load_tree_request)
            if not load_tree_response.success:
                self.get_logger().error(
                    f"could not load default tree: {load_tree_response.error_message}"
                )
            else:
                control_tree_execution_request = ControlTreeExecution.Request(
                    command=default_tree_control_command,
                    tick_frequency_hz=default_tree_tick_frequency_hz,
                )
                control_tree_execution_response = self.tree_manager.control_execution(
                    control_tree_execution_request
                )
                if not control_tree_execution_response.success:
                    self.get_logger().error(
                        f"could not execute default tree: "
                        f"{control_tree_execution_response.error_message}"
                    )

        # self.get_logger().info("initializing migration manger ...")
        # self.migration_manager = MigrationManager(tree_manager=self.tree_manager)

        # self.check_node_versions_service = rospy.Service(
        #    "~/check_node_versions", MigrateTree, check_node_versions
        # )

        # self.migrate_tree_service = rospy.Service(
        #    "~/migrate_tree", MigrateTree, self.migration_manager.migrate_tree
        # )
        # self.get_logger().info("initialized migration manager")

        self.get_logger().info("initializing package manager...")
        self.message_list_pub = self.create_publisher(
            Messages,
            "~/messages",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.packages_list_pub = self.create_publisher(
            Packages,
            "~/packages",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.package_manager = PackageManager(
            publish_message_list_callback=self.message_list_pub,
            publish_packages_list_callback=self.packages_list_pub,
        )

        self.package_manager.publish_packages_list()
        self.get_message_fields_service = self.create_service(
            GetMessageFields,
            "~/get_message_fields",
            callback=self.package_manager.get_message_fields,
            callback_group=self.service_callback_group,
        )
        self.get_message_constant_fields_service = self.create_service(
            GetMessageFields,
            "~/get_message_constant_fields",
            callback=self.package_manager.get_message_constant_fields_handler,
            callback_group=self.service_callback_group,
        )
        self.get_package_structure_service = self.create_service(
            GetPackageStructure,
            "~/get_package_structure",
            callback=self.package_manager.get_package_structure,
            callback_group=self.service_callback_group,
        )
        self.save_tree_service = self.create_service(
            SaveTree,
            "~/save_tree",
            callback=self.package_manager.save_tree,
            callback_group=self.service_callback_group,
        )

        self.package_manager.publish_message_list()
        self.get_logger().info("initialized package manager")
        self.get_logger().info("initialized tree node")

    def shutdown(self):
        """Shut down tree node in a safe way."""
        if self.tree_manager.get_state() not in [Tree.IDLE, Tree.EDITABLE, Tree.ERROR]:
            self.get_logger().info("Shutting down Behavior Tree")
            response = self.tree_manager.control_execution(
                ControlTreeExecution.Request(
                    command=ControlTreeExecution.Request.SHUTDOWN
                )
            )
            if not get_success(response):
                self.get_logger().error(
                    f"Failed to shut down Behavior Tree: {get_error_message(response)}"
                )


def main(argv=None):
    rclpy.init(args=argv)
    try:

        tree_node = TreeNode()

        executor = MultiThreadedExecutor()
        executor.add_node(tree_node)

        try:
            executor.spin()
        finally:
            executor.shutdown()
            tree_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    import sys

    main(sys.argv)
