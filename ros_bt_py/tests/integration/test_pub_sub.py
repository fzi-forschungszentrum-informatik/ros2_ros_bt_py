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

import rclpy

import ament_index_python
import os
import time

from rclpy.node import Node
from rclpy.client import Client

from ros_bt_py_interfaces.msg import (
    NodeState,
    TreeState,
    TreeStateList,
)
from ros_bt_py_interfaces.srv import (
    ControlTreeExecution,
    LoadTreeFromPath,
)

from conftest import launch_description


@pytest.mark.launch(fixture=launch_description)
def test_pub_sub(
    node: Node,
    load_client: Client,
    execute_client: Client,
    state_list: list[TreeStateList],
):
    tree_path = os.path.join(
        ament_index_python.get_package_share_directory("ros_bt_py"),
        "trees",
        "pub_sub_test.yaml",
    )
    load_req = LoadTreeFromPath.Request(
        path=f"file://{tree_path}",
        permissive=False,
    )
    load_future = load_client.call_async(load_req)
    rclpy.spin_until_future_complete(node, load_future, timeout_sec=30)
    assert load_future.done()
    assert load_future.result().success

    run_req = ControlTreeExecution.Request(
        command=ControlTreeExecution.Request.TICK_UNTIL_RESULT,
        tick_frequency_hz=10.0,
    )
    assert execute_client.wait_for_service(timeout_sec=30)
    run_future = execute_client.call_async(run_req)
    rclpy.spin_until_future_complete(node, run_future, timeout_sec=30)
    assert run_future.done()
    assert run_future.result().success

    # Check in a loop if tree is in state IDLE (has run and completed)
    start_time = time.time()
    while start_time + 60 > time.time():
        if state_list[-1].tree_states[0].state == TreeState.IDLE:
            break
        rclpy.spin_once(node, timeout_sec=10)
    final_state_msg = state_list[-1].tree_states[0]
    assert final_state_msg.state == TreeState.IDLE
    for node_state in final_state_msg.node_states:
        assert node_state.state == NodeState.SUCCEEDED
