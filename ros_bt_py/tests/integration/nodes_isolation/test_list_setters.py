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
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from ros_bt_py.vendor.result import Ok
from tests.integration.conftest import TreeControlNode, standard_tree_node


def _node_states(tree_control_node: TreeControlNode) -> dict[str, int]:
    match tree_control_node.get_tree_state():
        case Ok(state):
            return {
                node_state.node_id: node_state.state for node_state in state.node_states
            }
        case result:
            assert False, result.unwrap_err()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="ListSetters")
def test_list_and_dictionary_transformations_succeed(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/list_transformations.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    node_states = _node_states(tree_control_node)
    assert all(
        node_states[node_id] == NodeState.SUCCEEDED
        for node_id in (
            "30000000-0000-0000-0000-000000000002",
            "30000000-0000-0000-0000-000000000003",
            "30000000-0000-0000-0000-000000000004",
            "30000000-0000-0000-0000-000000000005",
        )
    )


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="IsInList")
def test_is_in_list_reports_success_and_failure(tree_control_node: TreeControlNode):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/list_membership.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    node_states = _node_states(tree_control_node)
    assert node_states["20000000-0000-0000-0000-000000000002"] == NodeState.SUCCEEDED
    assert node_states["20000000-0000-0000-0000-000000000003"] == NodeState.FAILED


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="IterateList")
def test_iterate_list_succeeds_for_an_empty_list(tree_control_node: TreeControlNode):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/iterate_list_empty.yaml"
    ).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    assert (
        _node_states(tree_control_node)["40000000-0000-0000-0000-000000000001"]
        == NodeState.SUCCEEDED
    )


@pytest.mark.launch(fixture=standard_tree_node)
def test_iterate_list_propagates_child_failure_and_resets(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/iterate_list_child_failure.yaml"
    ).is_ok()

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert (
        _node_states(tree_control_node)["50000000-0000-0000-0000-000000000001"]
        == NodeState.RUNNING
    )

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert (
        _node_states(tree_control_node)["50000000-0000-0000-0000-000000000001"]
        == NodeState.FAILED
    )

    assert tree_control_node.execute_tree(ControlTreeExecution.Request.RESET).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert (
        _node_states(tree_control_node)["50000000-0000-0000-0000-000000000001"]
        == NodeState.RUNNING
    )
