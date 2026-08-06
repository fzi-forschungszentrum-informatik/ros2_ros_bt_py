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

from threading import Thread
import time

import pytest
from example_interfaces.srv import Trigger
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from ros_bt_py.vendor.result import Ok
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution
from tests.integration.conftest import (
    TimeControlNode,
    TreeControlNode,
    sim_time_tree_node,
    standard_tree_node,
)


class WaitForServiceServer(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_service_server")
        self.create_service(Trigger, "/wait_for_service", lambda _, response: response)


@pytest.fixture
def wait_for_service_server():
    node = WaitForServiceServer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin)
    thread.start()
    yield node
    executor.shutdown(timeout_sec=30)
    thread.join(timeout=30)
    node.destroy_node()


def node_states(tree_control_node: TreeControlNode) -> dict[str, int]:
    match tree_control_node.get_node_states_by_name():
        case Ok(states):
            return states
        case result:
            assert False, result.unwrap_err()


@pytest.mark.launch(fixture=sim_time_tree_node)
def test_wait_for_service_times_out_with_simulated_time(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(10)
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/wait_for_service.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["WaitForService"] == NodeState.RUNNING

    time_control_node.set_time(12)
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["WaitForService"] == NodeState.FAILED


@pytest.mark.launch(fixture=sim_time_tree_node)
def test_wait_for_service_succeeds_when_service_is_available(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
    wait_for_service_server: WaitForServiceServer,
):
    del wait_for_service_server
    time_control_node.set_time(10)
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/wait_for_service.yaml"
    ).is_ok()

    for _ in range(10):
        assert tree_control_node.execute_tree(
            ControlTreeExecution.Request.TICK_ONCE
        ).is_ok()
        if node_states(tree_control_node)["WaitForService"] == NodeState.SUCCEEDED:
            return
        time.sleep(0.1)
    assert False, "WaitForService did not discover the available service"


@pytest.mark.launch(fixture=sim_time_tree_node)
def test_throttle_skips_child_until_interval_elapses(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(10)
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/throttle.yaml"
    ).is_ok()

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    states = node_states(tree_control_node)
    assert states["Throttle"] == NodeState.SUCCEEDED
    assert states["GetTimeNow"] == NodeState.IDLE

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["GetTimeNow"] == NodeState.IDLE

    time_control_node.set_time(12)
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["GetTimeNow"] == NodeState.IDLE


@pytest.mark.launch(fixture=sim_time_tree_node)
def test_get_time_now_uses_tree_node_clock(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(42)
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/get_time_now.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["GetTimeNow"] == NodeState.SUCCEEDED


@pytest.mark.launch(fixture=standard_tree_node)
def test_ros_param_reads_tree_node_parameter(tree_control_node: TreeControlNode):
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/ros_param.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert node_states(tree_control_node)["RosParam"] == NodeState.SUCCEEDED
