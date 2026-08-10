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

import json

import pytest
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from ros_bt_py.vendor.result import Ok
from tests.integration.conftest import (
    TimeControlNode,
    TreeControlNode,
    sim_time_tree_node,
    standard_tree_node,
)


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="Constant")
def test_constant_output_is_propagated_through_a_tree(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/constant_passthrough.yaml"
    ).is_ok()
    assert tree_control_node.set_publish_data(True).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    match tree_control_node.get_node_states_by_name():
        case Ok(node_states):
            pass
        case result:
            assert False, result.unwrap_err()

    assert node_states["Sequence"] == NodeState.SUCCEEDED
    assert node_states["Constant"] == NodeState.SUCCEEDED
    assert node_states["Passthrough"] == NodeState.SUCCEEDED

    match tree_control_node.get_tree_data():
        case Ok(data):
            wiring_data = data.wiring_data
        case result:
            assert False, result.unwrap_err()

    assert len(wiring_data) == 1
    assert wiring_data[0].is_current
    assert json.loads(wiring_data[0].serialized_data) == 42


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="StringConcatenation")
def test_string_concatenation_output_is_consumed_by_a_later_tree_node(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/string_concatenation.yaml"
    ).is_ok()
    assert tree_control_node.set_publish_data(True).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    match tree_control_node.get_tree_data():
        case Ok(data):
            wiring_data = data.wiring_data
        case result:
            assert False, result.unwrap_err()

    assert len(wiring_data) == 1
    assert wiring_data[0].is_current
    assert json.loads(wiring_data[0].serialized_data) == "hello world"


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="GetFileExtension")
def test_get_file_extension_output_is_propagated_through_a_tree(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/get_file_extension.yaml"
    ).is_ok()
    assert tree_control_node.set_publish_data(True).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    match tree_control_node.get_tree_data():
        case Ok(data):
            wiring_data = data.wiring_data
        case result:
            assert False, result.unwrap_err()

    assert len(wiring_data) == 2
    assert all(data.is_current for data in wiring_data)
    assert {json.loads(data.serialized_data) for data in wiring_data} == {
        "/tmp/archive.tar",
        ".gz",
    }


@pytest.mark.launch(fixture=sim_time_tree_node)
@pytest.mark.dependency(name="Wait")
def test_wait_transitions_with_simulated_time_and_restarts_after_reset(
    tree_control_node: TreeControlNode,
    time_control_node: TimeControlNode,
):
    time_control_node.set_time(10)
    assert tree_control_node.load_tree("trees/ros_nodes_isolation/wait.yaml").is_ok()

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    match tree_control_node.get_tree_state():
        case Ok(state):
            assert state.node_states[0].state == NodeState.RUNNING
        case result:
            assert False, result.unwrap_err()

    time_control_node.set_time(12)
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    match tree_control_node.get_tree_state():
        case Ok(state):
            assert state.node_states[0].state == NodeState.SUCCEEDED
        case result:
            assert False, result.unwrap_err()

    assert tree_control_node.execute_tree(ControlTreeExecution.Request.RESET).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    match tree_control_node.get_tree_state():
        case Ok(state):
            assert state.node_states[0].state == NodeState.RUNNING
        case result:
            assert False, result.unwrap_err()
