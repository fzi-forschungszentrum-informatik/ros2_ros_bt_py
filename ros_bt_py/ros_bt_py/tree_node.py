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
"""Module containing the main node for a ros_bt_py instance running the BT."""

from typeguard import typechecked
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ros_bt_py.parameters import tree_node_parameters
from ros_bt_py_interfaces.msg import (
    MessageChannels,
    TreeStructure,
    TreeStructureList,
    TreeState,
    TreeStateList,
    TreeData,
    TreeDataList,
    MessageTypes,
    Packages,
)
from ros_bt_py_interfaces.srv import (
    AddNode,
    AddNodeAtIndex,
    ControlTreeExecution,
    RemoveNode,
    WireNodeData,
    GetAvailableNodes,
    SetOptions,
    LoadTree,
    LoadTreeFromPath,
    MoveNode,
    ReplaceNode,
    GetSubtree,
    ClearTree,
    MorphNode,
    SaveTree,
    GetMessageFields,
    GetMessageConstantFields,
    GetPackageStructure,
    GenerateSubtree,
    ReloadTree,
    ChangeTreeName,
    GetFolderStructure,
    GetStorageFolders,
)

from std_srvs.srv import SetBool

from ros_bt_py.tree_manager import (
    TreeManager,
    get_success,
    get_error_message,
    get_available_nodes,
)
from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.package_manager import PackageManager

from ros_bt_py.ros_helpers import publish_message_channels
from functools import partial


class TreeNode(Node):
    """ROS node running a single behavior tree."""

    @typechecked
    def init_publisher(self) -> None:
        self.publisher_callback_group = ReentrantCallbackGroup()
        self.tree_structure_pub = self.create_publisher(
            TreeStructureList,
            "~/tree_structure_list",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.tree_state_pub = self.create_publisher(
            TreeStateList,
            "~/tree_state_list",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.tree_data_pub = self.create_publisher(
            TreeDataList,
            "~/tree_data_list",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # self.subtree_info_pub = self.create_publisher(
        #    SubtreeInfo,
        #    "~/debug/subtree_info",
        #    callback_group=self.publisher_callback_group,
        #    qos_profile=QoSProfile(
        #        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        #        history=QoSHistoryPolicy.KEEP_LAST,
        #        depth=1,
        #    ),
        # )
        self.node_diagnostics_pub = self.create_publisher(
            DiagnosticStatus,
            "~/debug/node_diagnostics",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.tick_frequency_pub = self.create_publisher(
            Float64,
            "~/debug/tick_frequency",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.ros_diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",  # TODO Should this be '~/diagnostics'?
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

    @typechecked
    def init_package_manager(self, params: tree_node_parameters.Params) -> None:
        self.get_logger().info("initializing package manager...")
        self.message_list_pub = self.create_publisher(
            MessageTypes,
            "~/message_types",
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
            tree_storage_directory_paths=params.tree_storage_paths,
            publish_message_list_callback=self.message_list_pub,
            publish_packages_list_callback=self.packages_list_pub,
        )

        self.package_manager.publish_packages_list()

        self.package_manager_service_callback_group = ReentrantCallbackGroup()

        self.get_message_fields_service = self.create_service(
            GetMessageFields,
            "~/get_message_fields",
            callback=self.package_manager.get_message_fields,
            callback_group=self.package_manager_service_callback_group,
        )
        self.get_message_constant_fields_service = self.create_service(
            GetMessageConstantFields,
            "~/get_message_constant_fields",
            callback=self.package_manager.get_message_constant_fields_handler,
            callback_group=self.package_manager_service_callback_group,
        )
        self.get_package_structure_service = self.create_service(
            GetPackageStructure,
            "~/get_package_structure",
            callback=self.package_manager.get_installed_package_structure,
            callback_group=self.package_manager_service_callback_group,
        )
        self.get_folder_structure_service = self.create_service(
            GetFolderStructure,
            "~/get_folder_structure",
            callback=self.package_manager.get_folder_structure,
            callback_group=self.package_manager_service_callback_group,
        )
        self.save_tree_service = self.create_service(
            SaveTree,
            "~/save_tree",
            callback=self.package_manager.save_tree_to_path,
            callback_group=self.package_manager_service_callback_group,
        )
        self.storage_folders_service = self.create_service(
            GetStorageFolders,
            "~/get_storage_folders",
            callback=self.package_manager.get_storage_folders,
            callback_group=self.package_manager_service_callback_group,
        )

        self.package_manager.publish_message_list()
        self.get_logger().info("initialized package manager")

    @typechecked
    def init_tree_manager(self, params: tree_node_parameters.Params) -> None:
        self.debug_manager = DebugManager(
            ros_node=self,
            node_diagnostics_publish_callback=self.node_diagnostics_pub.publish,
        )
        self.subtree_manager = SubtreeManager()
        self.tree_manager = TreeManager(
            ros_node=self,
            module_list=params.node_modules,
            debug_manager=self.debug_manager,
            subtree_manager=self.subtree_manager,
            publish_tree_structure_callback=self.tree_structure_pub.publish,
            publish_tree_state_callback=self.tree_state_pub.publish,
            publish_tree_data_callback=self.tree_data_pub.publish,
            publish_diagnostic_callback=self.ros_diagnostics_pub.publish,
            publish_tick_frequency_callback=self.tick_frequency_pub.publish,
            diagnostics_frequency=params.diagnostics_frequency_hz,
        )
        self.set_collect_node_diagnostics_service = self.create_service(
            SetBool,
            "~/debug/set_collect_node_diagnostics",
            callback=self.debug_manager.set_collect_node_diagnostics,
        )

        self.tree_manager_service_callback_group = ReentrantCallbackGroup()

        self.add_node_service = self.create_service(
            AddNode,
            "~/add_node",
            callback=self.tree_manager.add_node,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.add_node_at_index_service = self.create_service(
            AddNodeAtIndex,
            "~/add_node_at_index",
            callback=self.tree_manager.add_node_at_index,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.remove_node_service = self.create_service(
            RemoveNode,
            "~/remove_node",
            callback=self.tree_manager.remove_node,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.morph_node_service = self.create_service(
            MorphNode,
            "~/morph_node",
            callback=self.tree_manager.morph_node,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.wire_data_service = self.create_service(
            WireNodeData,
            "~/wire_data",
            callback=self.tree_manager.wire_data,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.unwire_data_service = self.create_service(
            WireNodeData,
            "~/unwire_data",
            callback=self.tree_manager.unwire_data,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.control_tree_execution_service = self.create_service(
            ControlTreeExecution,
            "~/control_tree_execution",
            callback=self.tree_manager.control_execution,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.get_available_nodes_service = self.create_service(
            GetAvailableNodes,
            "~/get_available_nodes",
            callback=get_available_nodes,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.get_subtree_service = self.create_service(
            GetSubtree,
            "~/get_subtree",
            callback=self.tree_manager.get_subtree,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.generate_subtree_service = self.create_service(
            GenerateSubtree,
            "~/generate_subtree",
            callback=self.tree_manager.generate_subtree,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.set_publish_subtrees_service = self.create_service(
            SetBool,
            "~/debug/set_publish_subtrees",
            callback=self.tree_manager.set_publish_subtrees,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.set_publish_data_service = self.create_service(
            SetBool,
            "~/debug/set_publish_data",
            callback=self.tree_manager.set_publish_data,
            callback_group=self.tree_manager_service_callback_group,
        )

        self.set_options_service = self.create_service(
            SetOptions,
            "~/set_options",
            callback=self.tree_manager.set_options,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.move_node_service = self.create_service(
            MoveNode,
            "~/move_node",
            callback=self.tree_manager.move_node,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.replace_node_service = self.create_service(
            ReplaceNode,
            "~/replace_node",
            callback=self.tree_manager.replace_node,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.load_tree_service = self.create_service(
            LoadTree,
            "~/load_tree",
            callback=self.tree_manager.load_tree,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.load_tree_from_path_service = self.create_service(
            LoadTreeFromPath,
            "~/load_tree_from_path",
            callback=self.tree_manager.load_tree_from_path,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.clear_service = self.create_service(
            ClearTree,
            "~/clear",
            callback=self.tree_manager.clear,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.reload_service = self.create_service(
            ReloadTree,
            "~/reload",
            callback=self.tree_manager.reload_tree,
            callback_group=self.tree_manager_service_callback_group,
        )
        self.change_tree_name_service = self.create_service(
            ChangeTreeName,
            "~/change_tree_name",
            callback=self.tree_manager.change_tree_name,
            callback_group=self.tree_manager_service_callback_group,
        )

        self.get_logger().info("initialized tree manager")

    def init_channels_publisher(self):
        self.message_channels_pub = self.create_publisher(
            MessageChannels,
            "~/message_channels",
            callback_group=self.publisher_callback_group,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )
        self.publish_channels_timer = self.create_timer(
            10.0,
            partial(
                publish_message_channels,
                self,
                self.message_channels_pub,
            ),
        )

    @typechecked
    def load_default_tree(self, params: tree_node_parameters.Params) -> None:
        if params.default_tree.load_default_tree:
            self.get_logger().info(
                f"loading default tree: {params.default_tree.tree_path}"
            )
            tree = TreeStructure(path=params.default_tree.tree_path)
            load_tree_request = LoadTree.Request(
                tree=tree, permissive=params.default_tree.load_default_tree_permissive
            )
            load_tree_response = LoadTree.Response()
            load_tree_response = self.tree_manager.load_tree(
                load_tree_request, load_tree_response
            )
            if not load_tree_response.success:
                self.get_logger().error(
                    f"could not load default tree: {load_tree_response.error_message}"
                )
            else:
                control_tree_execution_request = ControlTreeExecution.Request(
                    command=params.default_tree.control_command,
                    tick_frequency_hz=params.default_tree.tick_frequency_hz,
                )
                control_tree_execution_response = ControlTreeExecution.Response()
                control_tree_execution_response = self.tree_manager.control_execution(
                    control_tree_execution_request, control_tree_execution_response
                )
                if not control_tree_execution_response.success:
                    self.get_logger().error(
                        f"could not execute default tree: "
                        f"{control_tree_execution_response.error_message}"
                    )

    @typechecked
    def shutdown(self) -> None:
        """Shut down tree node in a safe way."""
        if self.tree_manager.state not in [
            TreeState.IDLE,
            TreeState.EDITABLE,
            TreeState.ERROR,
        ]:
            self.get_logger().info("Shutting down Behavior Tree")
            response = self.tree_manager.control_execution(
                ControlTreeExecution.Request(
                    command=ControlTreeExecution.Request.SHUTDOWN
                ),
                ControlTreeExecution.Response(),
            )
            if not get_success(response):
                self.get_logger().error(
                    f"Failed to shut down Behavior Tree: {get_error_message(response)}"
                )


def main(argv=None):

    rclpy.init(args=argv)
    tree_node = TreeNode(node_name="BehaviorTreeNode")
    param_listener = tree_node_parameters.ParamListener(tree_node)
    params = param_listener.get_params()
    tree_node.init_publisher()
    tree_node.init_package_manager(params=params)
    tree_node.init_tree_manager(params=params)
    tree_node.load_default_tree(params=params)
    tree_node.init_channels_publisher()

    # NOTE This print statement is used by integration tests
    #   to determine when testing can begin
    # DO NOT MODIFY THIS
    print("Finished starting tree node", flush=True)

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(tree_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        get_logger("tree_node").fatal("Shutting down rclpy!")


if __name__ == "__main__":
    import sys

    main(sys.argv)
