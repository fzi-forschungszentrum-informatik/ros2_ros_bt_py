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


FILE_NODE_STATES = {
    "20000000-0000-0000-0000-000000000002": NodeState.SUCCEEDED,
    "20000000-0000-0000-0000-000000000003": NodeState.SUCCEEDED,
    "20000000-0000-0000-0000-000000000005": NodeState.FAILED,
    "20000000-0000-0000-0000-000000000007": NodeState.FAILED,
    "20000000-0000-0000-0000-000000000009": NodeState.FAILED,
}


def assert_file_node_states(tree_control_node: TreeControlNode):
    match tree_control_node.get_tree_state():
        case Ok(state):
            node_states = {
                node_state.node_id: node_state.state for node_state in state.node_states
            }
        case result:
            assert False, result.unwrap_err()

    for node_id, expected_state in FILE_NODE_STATES.items():
        assert node_states[node_id] == expected_state


@pytest.mark.launch(fixture=standard_tree_node)
def test_yaml_file_loaders_handle_success_failures_cache_and_reset(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/yaml_file_loaders.yaml"
    ).is_ok()

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert_file_node_states(tree_control_node)

    # No file path changes between ticks, so each node returns its cached result.
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert_file_node_states(tree_control_node)

    assert tree_control_node.execute_tree(ControlTreeExecution.Request.RESET).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    assert_file_node_states(tree_control_node)
