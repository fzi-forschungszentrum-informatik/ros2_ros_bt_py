# Copyright 2025 FZI Forschungszentrum Informatik
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
import pytest

import contextlib

import launch
import launch_pytest
import launch_ros
import launch_testing.actions
import launch_testing.event_handlers
import rclpy

# ROS2 versions starting from Kilted have a EnableRmwIsolation launch Action
#   Use that instead of this custom implementation once available
import os
import domain_coordinator

from rclpy.node import Node

from ros_bt_py_interfaces.msg import (
    TreeStateList,
)
from ros_bt_py_interfaces.srv import (
    ControlTreeExecution,
    LoadTreeFromPath,
)


# This function specifies the processes to be run for our test.
@launch_pytest.fixture
def launch_description():
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
            additional_env={"PYTHONUNBUFFERED": "1"},
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


@pytest.fixture
def node():
    rclpy.init()
    node = rclpy.create_node("test_node")
    yield node
    node.destroy_node()
    rclpy.shutdown()


@pytest.fixture
def load_client(node: Node):
    return node.create_client(
        LoadTreeFromPath,
        "/BehaviorTreeNode/load_tree_from_path",
    )


@pytest.fixture
def execute_client(node: Node):
    return node.create_client(
        ControlTreeExecution,
        "/BehaviorTreeNode/control_tree_execution",
    )


@pytest.fixture
def state_list(node: Node):
    state_msg_list: list[TreeStateList] = []

    def save_state(msg: TreeStateList):
        state_msg_list.append(msg)

    _ = node.create_subscription(
        TreeStateList,
        "/BehaviorTreeNode/tree_state_list",
        save_state,
        1,
    )
    return state_msg_list
