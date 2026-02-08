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
  # "action/ExecuteRemoteCapability.action"
  # "action/RunTree.action"
  # "action/FindBestExecutor.action"


  # "msg/CapabilityExecutionStatus.msg"
  # "msg/CapabilityImplementation.msg"
  "msg/CapabilityInterface.msg"
  # "msg/CapabilityIOBridgeData.msg"

  "msg/NodeOption.msg"
  "msg/NodeIO.msg"
  "msg/NodeDataLocation.msg"
  "msg/DocumentedNode.msg"
  "msg/NodeStructure.msg"
  "msg/NodeState.msg"
  "msg/Wiring.msg"
  "msg/WiringData.msg"
  "msg/TreeStructure.msg"
  "msg/TreeState.msg"
  "msg/TreeData.msg"
  "msg/TreeStructureList.msg"
  "msg/TreeStateList.msg"
  "msg/TreeDataList.msg"
  # "msg/TreeDataUpdate.msg"

  "msg/UtilityBounds.msg"
  "msg/MessageChannel.msg"
  "msg/MessageChannels.msg"
  "msg/MessageTypes.msg"
  "msg/Package.msg"
  "msg/Packages.msg"
  # "msg/PingMsg.msg"
  # "msg/Precondition.msg"
  # "msg/RemoteCapabilitySlotStatus.msg"
  # "msg/RemoteSlotState.msg"

  "msg/BTLogMessage.msg"
  "srv/SetLogLevel.srv"

  "srv/AddNode.srv"
  "srv/AddNodeAtIndex.srv"
  "srv/ChangeTreeName.srv"
  "srv/ClearTree.srv"
  "srv/ControlTreeExecution.srv"
  # "srv/EvaluateUtility.srv"
  "srv/GenerateSubtree.srv"
  "srv/GetAvailableNodes.srv"
  "srv/GetAvailableSubtrees.srv"
  "srv/GetFolderStructure.srv"
  "srv/GetMessageFields.srv"
  "srv/GetMessageConstantFields.srv"
  "srv/GetPackageStructure.srv"
  "srv/GetStorageFolders.srv"
  "srv/GetSubtree.srv"
  "srv/InsertNode.srv"
  "srv/LoadTree.srv"
  "srv/LoadTreeFromPath.srv"
  "srv/MigrateTree.srv"
  "srv/MorphNode.srv"
  "srv/MoveNode.srv"
  "srv/ReloadTree.srv"
  "srv/RemoveNode.srv"
  "srv/ReplaceNode.srv"
  "srv/SaveTree.srv"
  "srv/SetOptions.srv"
  "srv/SetSimulateTick.srv"
  "srv/TestService.srv"
  "srv/WireNodeData.srv"

  # "srv/capabilities/CancelRemoteCapabilitySlot.srv"
  # "srv/capabilities/CheckPreconditionStatus.srv"
  # "srv/capabilities/DeleteCapabilityImplementation.srv"
  # "srv/capabilities/FindBestCapabilityExecutor.srv"
  # "srv/capabilities/GetAvailableRemoteCapabilitySlots.srv"
  # "srv/capabilities/GetCapabilityImplementations.srv"
  # "srv/capabilities/GetCapabilityInterfaces.srv"
  # "srv/capabilities/GetLocalBid.srv"
  # "srv/capabilities/LoadCapabilities.srv"
  # "srv/capabilities/PrepareLocalImplementation.srv"
  # "srv/capabilities/PutCapabilityImplementation.srv"
  # "srv/capabilities/PutCapabilityInterfaces.srv"
  # "srv/capabilities/RequestCapabilityExecution.srv"
  # "srv/capabilities/ReserveRemoteCapabilitySlot.srv"
  # "srv/capabilities/RunRemoteCapabilitySlot.srv"
  # "srv/capabilities/SaveCapabilities.srv"

  DEPENDENCIES builtin_interfaces
)


ament_package()
