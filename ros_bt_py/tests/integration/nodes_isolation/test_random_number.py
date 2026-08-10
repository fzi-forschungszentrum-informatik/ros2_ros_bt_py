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


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(name="RandomInt")
def test_random_int_reports_success_and_rejects_equal_bounds(
    tree_control_node: TreeControlNode,
):
    assert tree_control_node.load_tree(
        "trees/nodes_isolation/random_number.yaml"
    ).is_ok()
    result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert result.is_err()
    assert "cannot be equal or greater" in result.unwrap_err()

    match tree_control_node.get_node_states_by_name():
        case Ok(states):
            pass
        case result:
            assert False, result.unwrap_err()

    assert states["RandomInRange"] == NodeState.SUCCEEDED
