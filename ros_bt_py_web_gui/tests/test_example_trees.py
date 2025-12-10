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
import unittest

import time

import launch
import launch_ros
import launch_testing.actions
import launch_testing.event_handlers
import rclpy

import ament_index_python
import os

from ros_bt_py_interfaces.msg import (
    NodeState,
    TreeState,
    TreeStateList,
)
from ros_bt_py_interfaces.srv import (
    ControlTreeExecution,
    LoadTreeFromPath,
)


def generate_test_description():
    tree_node = launch_ros.actions.Node(
        package="ros_bt_py",
        executable="tree_node",
        additional_env={"PYTHONUNBUFFERED": "1"},
    )
    ld = launch.LaunchDescription(
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
    return ld, {"tree_node": tree_node}


class TestBehaviorTrees(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def save_state(self, msg: TreeStateList):
        self.state_messages.append(msg)

    def setUp(self):
        self.node = rclpy.create_node("test_tree")
        self.load_client = self.node.create_client(
            LoadTreeFromPath,
            "/BehaviorTreeNode/load_tree_from_path",
        )
        self.execute_client = self.node.create_client(
            ControlTreeExecution,
            "/BehaviorTreeNode/control_tree_execution",
        )
        self.state_messages: list[TreeStateList] = []
        self.state_subscriber = self.node.create_subscription(
            TreeStateList,
            "/BehaviorTreeNode/tree_state_list",
            self.save_state,
            1,
        )

    def tearDown(self):
        self.node.destroy_node()

    def test_pub_sub(self, proc_output, tree_node):
        tree_path = os.path.join(
            ament_index_python.get_package_share_directory("ros_bt_py_web_gui"),
            "test_trees",
            "pub_sub_test.yaml",
        )
        load_req = LoadTreeFromPath.Request(
            path=f"file://{tree_path}",
            permissive=False,
        )
        load_future = self.load_client.call_async(load_req)
        rclpy.spin_until_future_complete(self.node, load_future, timeout_sec=30)
        assert load_future.done()
        assert load_future.result().success

        run_req = ControlTreeExecution.Request(
            command=ControlTreeExecution.Request.TICK_UNTIL_RESULT,
            tick_frequency_hz=10.0,
        )
        assert self.execute_client.wait_for_service(timeout_sec=30)
        run_future = self.execute_client.call_async(run_req)
        rclpy.spin_until_future_complete(self.node, run_future, timeout_sec=30)
        assert run_future.done()
        assert run_future.result().success

        # Check in a loop if tree is in state IDLE (has run and completed)
        start_time = time.time()
        while start_time + 60 > time.time():
            if self.state_messages[-1].tree_states[0].state == TreeState.IDLE:
                break
            rclpy.spin_once(self.node, timeout_sec=10)
        final_state_msg = self.state_messages[-1].tree_states[0]
        assert final_state_msg.state == TreeState.IDLE
        for node_state in final_state_msg.node_states:
            assert node_state.state == NodeState.SUCCEEDED
