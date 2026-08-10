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
from tests.integration.conftest import TreeControlNode, standard_tree_node


@pytest.mark.launch(fixture=standard_tree_node)
def test_message_fields_round_trip(tree_control_node: TreeControlNode):
    assert tree_control_node.load_tree(
        "trees/ros_nodes_isolation/message_converters.yaml"
    ).is_ok()
    assert tree_control_node.set_publish_data(True).is_ok()
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    match tree_control_node.get_tree_state():
        case Ok(state):
            assert all(node.state == NodeState.SUCCEEDED for node in state.node_states)
        case result:
            assert False, result.unwrap_err()

    match tree_control_node.get_tree_data():
        case Ok(data):
            assert len(data.wiring_data) == 3
            assert all(wiring.is_current for wiring in data.wiring_data)
            assert {
                json.dumps(json.loads(wiring.serialized_data), sort_keys=True)
                for wiring in data.wiring_data
            } == {'"round trip"', '{"data": "round trip"}'}
        case result:
            assert False, result.unwrap_err()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.parametrize(
    "tree_file",
    [
        "trees/ros_nodes_isolation/message_to_fields_missing_input.yaml",
        "trees/ros_nodes_isolation/fields_to_message_missing_field.yaml",
    ],
)
def test_message_converters_fail_without_required_data(
    tree_control_node: TreeControlNode, tree_file: str
):
    assert tree_control_node.load_tree(tree_file).is_ok()
    result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert not result.is_ok()

    match tree_control_node.get_tree_state():
        case Ok(state):
            assert state.node_states[0].state == NodeState.BROKEN
        case result:
            assert False, result.unwrap_err()
