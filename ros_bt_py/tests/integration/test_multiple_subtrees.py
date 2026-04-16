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

import time

from rclpy.node import Node
from rclpy.client import Client

from std_srvs.srv import SetBool

from ros_bt_py_interfaces.msg import (
    NodeState,
    TreeState,
    TreeStructureList,
    TreeStateList,
)
from ros_bt_py_interfaces.srv import LoadTreeFromPath, ControlTreeExecution

from conftest import launch_description


@pytest.mark.launch(fixture=launch_description)
def test_multiple_subtrees(
    node: Node,
    pub_subtrees_client: Client,
    load_client: Client,
    execute_client: Client,
    structure_list: list[TreeStructureList],
    state_list: list[TreeStateList],
):
    pub_subtrees_req = SetBool.Request(data=True)
    assert pub_subtrees_client.wait_for_service(timeout_sec=30)
    pub_subtrees_future = pub_subtrees_client.call_async(pub_subtrees_req)
    rclpy.spin_until_future_complete(node, pub_subtrees_future, timeout_sec=30)
    assert pub_subtrees_future.done()
    assert pub_subtrees_future.result().success

    load_req = LoadTreeFromPath.Request(
        path="package://ros_bt_py/trees/double_nested_five_pub_sub_test.yaml",
        permissive=False,
    )
    assert load_client.wait_for_service(timeout_sec=30)
    load_future = load_client.call_async(load_req)
    rclpy.spin_until_future_complete(node, load_future, timeout_sec=120)
    assert load_future.done()
    assert load_future.result().success

    # Check in a loop if we receive more than one tree structure
    start_time = time.time()
    while start_time + 60 > time.time():
        if len(structure_list[-1].tree_structures) > 1:
            break
        rclpy.spin_once(node, timeout_sec=10)

    tree_id_set: set[str] = set()
    tree_structure_list = structure_list[-1].tree_structures
    for tree in tree_structure_list:
        assert tree.tree_id not in tree_id_set
        tree_id_set.add(tree.tree_id)

    run_req = ControlTreeExecution.Request(
        command=ControlTreeExecution.Request.TICK_UNTIL_RESULT,
        tick_frequency_hz=10.0,
    )
    assert execute_client.wait_for_service(timeout_sec=30)
    run_future = execute_client.call_async(run_req)
    rclpy.spin_until_future_complete(node, run_future, timeout_sec=60)
    assert run_future.done()
    assert run_future.result().success

    # Check in a loop if tree is in state IDLE (has run and completed)
    start_time = time.time()
    while start_time + 60 > time.time():
        all_trees_idle = True
        for tree in state_list[-1].tree_states:
            if tree.state != TreeState.IDLE:
                all_trees_idle = False
        if all_trees_idle:
            break
        rclpy.spin_once(node, timeout_sec=10)
    final_state_msg = state_list[-1]
    for tree_state in final_state_msg.tree_states:
        for node_state in tree_state.node_states:
            assert node_state.state == NodeState.SUCCEEDED
