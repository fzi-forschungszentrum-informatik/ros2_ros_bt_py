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

from ros_bt_py.vendor.result import Err, Ok

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import TreeControlNode, standard_tree_node


def node_states(tree_state):
    return {
        str(node_state.node_id): node_state.state
        for node_state in tree_state.node_states
    }


@pytest.mark.launch(fixture=standard_tree_node)
def test_subtree_nested_io_lifecycle_and_publication(
    tree_control_node: TreeControlNode,
):
    load_result = tree_control_node.load_tree(
        "trees/ros_nodes_isolation/subtree_outer.yaml"
    )
    assert load_result.is_ok()

    match tree_control_node.get_tree_structure():
        case Err(e):
            assert False, e
        case Ok(_):
            pass
    with tree_control_node._tree_msg_lock:
        structures = tree_control_node._tree_structure_msg.tree_structures  # type: ignore
    assert {structure.name for structure in structures} == {
        "subtree_outer",
        "OuterSubtree",
        "NestedSubtree",
    }

    assert tree_control_node.set_publish_data(True).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(_):
            pass
    with tree_control_node._tree_msg_lock:
        tree_states = tree_control_node._tree_state_msg.tree_states  # type: ignore
    nested_state = next(
        state
        for state in tree_states
        if any(str(node.node_id).startswith("10000000") for node in state.node_states)
    )
    states = node_states(nested_state)
    assert states["10000000-0000-0000-0000-000000000003"] == NodeState.SUCCEEDED
    assert states["10000000-0000-0000-0000-000000000004"] == NodeState.SUCCEEDED
    assert states["10000000-0000-0000-0000-000000000005"] == NodeState.FAILED
    assert states["10000000-0000-0000-0000-000000000006"] == NodeState.RUNNING

    match tree_control_node.get_tree_data():
        case Err(e):
            assert False, e
        case Ok(_):
            pass
    with tree_control_node._tree_msg_lock:
        tree_data = tree_control_node._tree_data_msg.tree_data  # type: ignore
    assert any(
        wiring.serialized_data == "42"
        and wiring.wiring.source.data_key.endswith(".out")
        for data in tree_data
        for wiring in data.wiring_data
    )

    assert tree_control_node.execute_tree(ControlTreeExecution.Request.STOP).is_ok()
    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(_):
            pass
    with tree_control_node._tree_msg_lock:
        tree_states = tree_control_node._tree_state_msg.tree_states  # type: ignore
    nested_state = next(
        state
        for state in tree_states
        if any(str(node.node_id).startswith("10000000") for node in state.node_states)
    )
    assert all(state == NodeState.IDLE for state in node_states(nested_state).values())

    assert tree_control_node.execute_tree(ControlTreeExecution.Request.RESET).is_ok()
    assert tree_control_node.execute_tree(ControlTreeExecution.Request.SHUTDOWN).is_ok()
    match tree_control_node.get_tree_state():
        case Err(e):
            assert False, e
        case Ok(_):
            pass
    with tree_control_node._tree_msg_lock:
        tree_states = tree_control_node._tree_state_msg.tree_states  # type: ignore
    nested_state = next(
        state
        for state in tree_states
        if any(str(node.node_id).startswith("10000000") for node in state.node_states)
    )
    assert all(
        state == NodeState.SHUTDOWN for state in node_states(nested_state).values()
    )
