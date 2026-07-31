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

import pytest

from typing import Optional

import contextlib
from threading import Lock
import time
import uuid

import launch
import launch_pytest
import launch_ros
import launch_testing.actions
import launch_testing.event_handlers

import rclpy
from rclpy.node import Node

# ROS2 versions starting from Kilted have a EnableRmwIsolation launch Action
#   Use that instead of this custom implementation once available
import os
import domain_coordinator

from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.ros_helpers import uuid_to_ros

from ros_bt_py_interfaces.msg import (
    TreeData,
    TreeDataList,
    TreeState,
    TreeStructure,
    TreeStateList,
    TreeStructureList,
)
from ros_bt_py_interfaces.srv import (
    ControlTreeExecution,
    LoadTreeFromPath,
)

from rosgraph_msgs.msg import Clock
from std_srvs.srv import SetBool


# This function specifies the processes to be run for our test.
def tree_launch_description(use_sim_time: bool):
    with contextlib.ExitStack() as stack:
        if (
            "ROS_DOMAIN_ID" not in os.environ
            and "DISABLE_ROS_ISOLATION" not in os.environ
        ):
            domain_id = stack.enter_context(domain_coordinator.domain_id())
            print("Running with ROS_DOMAIN_ID {}".format(domain_id))
            os.environ["ROS_DOMAIN_ID"] = str(domain_id)
        tree_node = launch_ros.actions.Node(
            package="ros_bt_py",
            executable="tree_node",
            parameters=[{"use_sim_time": use_sim_time}],
            additional_env={
                "PYTHONUNBUFFERED": "1"
            },  # Necessary for the ReadyListener below
        )
        yield launch.LaunchDescription(
            [
                tree_node,
                launch.actions.RegisterEventHandler(
                    launch_testing.event_handlers.StdoutReadyListener(
                        target_action=tree_node,
                        ready_txt="Finished starting tree node",
                        actions=[launch_testing.actions.ReadyToTest()],
                    )
                ),
            ]
        )


@launch_pytest.fixture
def standard_tree_node():
    yield from tree_launch_description(use_sim_time=False)


@launch_pytest.fixture
def sim_time_tree_node():
    yield from tree_launch_description(use_sim_time=True)


class TreeControlNode(Node):
    """
    Provides functions to easily interact with the BT under testing.

    Most of these functions use return values of type ``Result``.
    Make sure to check (assert) them in a way that allows the error message
    to be picked up, like ``assert result.is_ok()`` or
    ``match result:
        case Err(e):
            assert False, e
        case Ok(v):
            ...``
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("tree_control_node", *args, **kwargs)

        self._tree_msg_lock = Lock()
        self._tree_structure_msg: Optional[TreeStructureList] = None
        self._tree_state_msg: Optional[TreeStateList] = None
        self._tree_data_msg: Optional[TreeDataList] = None

        self.load_tree_client = self.create_client(
            LoadTreeFromPath,
            "/BehaviorTreeNode/load_tree_from_path",
        )
        self.execute_tree_client = self.create_client(
            ControlTreeExecution,
            "/BehaviorTreeNode/control_tree_execution",
        )
        self.set_publish_data_client = self.create_client(
            SetBool,
            "/BehaviorTreeNode/debug/set_publish_data",
        )
        self.tree_structure_subscription = self.create_subscription(
            TreeStructureList,
            "/BehaviorTreeNode/tree_structure_list",
            self._new_tree_structure_msg,
            1,
        )
        self.tree_state_subscription = self.create_subscription(
            TreeStateList,
            "/BehaviorTreeNode/tree_state_list",
            self._new_tree_state_msg,
            1,
        )
        self.tree_data_subscription = self.create_subscription(
            TreeDataList,
            "/BehaviorTreeNode/tree_data_list",
            self._new_tree_data_msg,
            1,
        )

    def _new_tree_structure_msg(self, msg: TreeStructureList):
        with self._tree_msg_lock:
            self._tree_structure_msg = msg

    def _new_tree_state_msg(self, msg: TreeStateList):
        with self._tree_msg_lock:
            self._tree_state_msg = msg

    def _new_tree_data_msg(self, msg: TreeDataList):
        with self._tree_msg_lock:
            self._tree_data_msg = msg

    def load_tree(self, tree_file: str, wait_time=30) -> Result[None, str]:
        with self._tree_msg_lock:
            self._tree_structure_msg = None
            self._tree_state_msg = None
            self._tree_data_msg = None
        load_req = LoadTreeFromPath.Request(
            path=f"package://ros_bt_py/{tree_file}",
            permissive=False,
        )
        if not self.load_tree_client.wait_for_service(timeout_sec=wait_time):
            return Err("Load tree server not available")
        load_future = self.load_tree_client.call_async(load_req)
        rclpy.spin_until_future_complete(self, load_future, timeout_sec=wait_time)
        if not load_future.done():
            return Err("Load tree request did not complete")
        if load_future.result().success:  # type: ignore
            return Ok(None)
        else:
            return Err(load_future.result().error_message)  # type: ignore

    def set_publish_data(self, enabled: bool, wait_time=30) -> Result[None, str]:
        if not self.set_publish_data_client.wait_for_service(timeout_sec=wait_time):
            return Err("Set publish data server not available")
        future = self.set_publish_data_client.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self, future, timeout_sec=wait_time)
        if not future.done():
            return Err("Set publish data request did not complete")
        if future.result().success:  # type: ignore
            return Ok(None)
        return Err(future.result().message)  # type: ignore

    def execute_tree(self, tree_action: int, wait_time=30) -> Result[None, str]:
        run_req = ControlTreeExecution.Request(
            command=tree_action,
            tick_frequency_hz=10.0,
        )
        if not self.execute_tree_client.wait_for_service(timeout_sec=wait_time):
            return Err("Execute tree server not available")
        run_future = self.execute_tree_client.call_async(run_req)
        rclpy.spin_until_future_complete(self, run_future, timeout_sec=wait_time)
        if not run_future.done():
            return Err("Execute tree request did not complete")
        if run_future.result().success:  # type: ignore
            return Ok(None)
        else:
            return Err(run_future.result().error_message)  # type: ignore

    def get_tree_structure(
        self, tree_id=uuid.UUID(int=0), wait_time=60
    ) -> Result[TreeStructure, str]:
        ros_tree_id = uuid_to_ros(tree_id)
        start_time = time.time()
        while start_time + wait_time > time.time():
            if self._tree_structure_msg is not None:
                with self._tree_msg_lock:
                    structure: TreeStructure
                    for structure in self._tree_structure_msg.tree_structures:
                        if structure.tree_id == ros_tree_id:
                            return Ok(structure)
            rclpy.spin_once(self, timeout_sec=5)
        return Err("No tree structure with this id")

    def get_tree_data(
        self, tree_id=uuid.UUID(int=0), wait_time=60
    ) -> Result[TreeData, str]:
        ros_tree_id = uuid_to_ros(tree_id)
        start_time = time.time()
        while start_time + wait_time > time.time():
            if self._tree_data_msg is not None:
                with self._tree_msg_lock:
                    for data in self._tree_data_msg.tree_data:
                        if data.tree_id == ros_tree_id:
                            return Ok(data)
            rclpy.spin_once(self, timeout_sec=5)
        return Err("No tree data with this id")

    def get_tree_state(
        self, tree_id=uuid.UUID(int=0), wait_time=60
    ) -> Result[TreeState, str]:
        ros_tree_id = uuid_to_ros(tree_id)
        start_time = time.time()
        while start_time + wait_time > time.time():
            if self._tree_state_msg is not None:
                with self._tree_msg_lock:
                    state: TreeState
                    for state in self._tree_state_msg.tree_states:
                        if state.tree_id == ros_tree_id:
                            return Ok(state)
            rclpy.spin_once(self, timeout_sec=5)
        return Err("No tree structure with this id")


@pytest.fixture
def tree_control_node():
    rclpy.init()
    node = TreeControlNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


class TimeControlNode(Node):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__("time_control_node", *args, **kwargs)

        self.clock_publisher = self.create_publisher(
            Clock,
            "/clock",
            1,
        )

    def set_time(self, seconds: int):
        clock_msg = Clock()
        clock_msg.clock.sec = seconds
        self.clock_publisher.publish(clock_msg)


@pytest.fixture
def time_control_node():
    # No rclpy init, we assume a tree_control_node to always be used
    node = TimeControlNode()
    yield node
    node.destroy_node()
