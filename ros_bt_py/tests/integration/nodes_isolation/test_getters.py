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


def get_node_states(tree_control_node: TreeControlNode) -> dict[str, int]:
    match tree_control_node.get_node_states_by_name():
        case Ok(states):
            return states
        case result:
            assert False, result.unwrap_err()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="GetListItem")
@pytest.mark.dependency(name="GetDictItem")
@pytest.mark.dependency(name="GetMultipleDictItems")
@pytest.mark.dependency(name="GetAttr")
def test_getter_nodes_extract_data_fail_on_invalid_lookups_and_forward_children(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree("trees/nodes_isolation/getters.yaml").is_ok()
    assert tree_control_node.set_publish_data(True).is_ok()

    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()
    states = get_node_states(tree_control_node)

    for node_name in (
        "SuccessfulGetterChild",
        "GetListItem",
        "GetDictItem",
        "GetMultipleDictItems",
        "GetAttr",
    ):
        assert states[node_name] == NodeState.SUCCEEDED
    for node_name in (
        "InvalidListItem",
        "InvalidDictItem",
        "InvalidMultipleDictItems",
        "InvalidAttr",
    ):
        assert states[node_name] == NodeState.FAILED

    match tree_control_node.get_tree_data():
        case Ok(data):
            wiring_data = data.wiring_data
        case result:
            assert False, result.unwrap_err()

    assert len(wiring_data) == 4
    assert all(data.is_current for data in wiring_data)
    assert {
        json.dumps(json.loads(data.serialized_data), sort_keys=True)
        for data in wiring_data
    } == {
        '"one"',
        '"Ada"',
        '"hello"',
        '["Ada", "Lovelace"]',
    }
