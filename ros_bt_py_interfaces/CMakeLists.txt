cmake_minimum_required(VERSION 3.8)
project(ros_bt_py_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ExecuteRemoteCapability.action"
  "action/RunTree.action"
  "action/FindBestExecutor.action"

  "msg/CapabilityExecutionStatus.msg"
  "msg/CapabilityImplementation.msg"
  "msg/CapabilityInterface.msg"
  "msg/CapabilityIOBridgeData.msg"
  "msg/DebugInfo.msg"
  "msg/DebugSettings.msg"
  "msg/DocumentedNode.msg"
  "msg/Message.msg"
  "msg/Messages.msg"
  "msg/Node.msg"
  "msg/NodeData.msg"
  "msg/NodeDataLocation.msg"
  "msg/NodeDataWiring.msg"
  "msg/NodeDiagnostics.msg"
  "msg/Package.msg"
  "msg/Packages.msg"
  "msg/PingMsg.msg"
  "msg/Precondition.msg"
  "msg/RemoteCapabilitySlotStatus.msg"
  "msg/RemoteSlotState.msg"
  "msg/Tree.msg"
  "msg/TreeDataUpdate.msg"
  "msg/UtilityBounds.msg"

  "srv/AddNode.srv"
  "srv/AddNodeAtIndex.srv"
  "srv/capabilities/CancelRemoteCapabilitySlot.srv"
  "srv/capabilities/CheckPreconditionStatus.srv"
  "srv/capabilities/DeleteCapabilityImplementation.srv"
  "srv/capabilities/FindBestCapabilityExecutor.srv"
  "srv/capabilities/GetAvailableRemoteCapabilitySlots.srv"
  "srv/capabilities/GetCapabilityImplementations.srv"
  "srv/capabilities/GetCapabilityInterfaces.srv"
  "srv/capabilities/GetLocalBid.srv"
  "srv/capabilities/LoadCapabilities.srv"
  "srv/capabilities/PrepareLocalImplementation.srv"
  "srv/capabilities/PutCapabilityImplementation.srv"
  "srv/capabilities/PutCapabilityInterfaces.srv"
  "srv/capabilities/RequestCapabilityExecution.srv"
  "srv/capabilities/ReserveRemoteCapabilitySlot.srv"
  "srv/capabilities/RunRemoteCapabilitySlot.srv"
  "srv/capabilities/SaveCapabilities.srv"
  "srv/ChangeTreeName.srv"
  "srv/ClearTree.srv"
  "srv/Continue.srv"
  "srv/ControlTreeExecution.srv"
  "srv/EvaluateUtility.srv"
  "srv/FixYaml.srv"
  "srv/GenerateSubtree.srv"
  "srv/GetAvailableNodes.srv"
  "srv/GetAvailableSubtrees.srv"
  "srv/GetMessageFields.srv"
  "srv/GetPackageStructure.srv"
  "srv/GetSubtree.srv"
  "srv/InsertNode.srv"
  "srv/LoadTree.srv"
  "srv/LoadTreeFromPath.srv"
  "srv/MigrateTree.srv"
  "srv/ModifyBreakpoints.srv"
  "srv/MorphNode.srv"
  "srv/MoveNode.srv"
  "srv/ReloadTree.srv"
  "srv/RemoveNode.srv"
  "srv/ReplaceNode.srv"
  "srv/SaveTree.srv"
  "srv/SetExecutionMode.srv"
  "srv/SetOptions.srv"
  "srv/SetSimulateTick.srv"
  "srv/TestService.srv"
  "srv/WireNodeData.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
