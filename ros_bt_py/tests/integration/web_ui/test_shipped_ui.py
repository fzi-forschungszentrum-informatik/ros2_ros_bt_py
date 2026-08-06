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

from playwright.sync_api import Page, expect

from ros_bt_py.vendor.result import Ok
from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.srv import ControlTreeExecution
from tests.integration.conftest import TreeControlNode


TREE_FILE = "trees/nodes_isolation/constant_passthrough.yaml"


def load_test_tree(tree_control_node: TreeControlNode) -> None:
    result = tree_control_node.load_tree(TREE_FILE)
    assert result.is_ok(), result


def tree_states(tree_control_node: TreeControlNode) -> dict[str, int]:
    result = tree_control_node.get_tree_state()
    assert isinstance(result, Ok), result
    return {state.node_id: state.state for state in result.unwrap().node_states}


def test_shipped_ui_connects_and_renders_tree(
    open_web_gui: Page, tree_control_node: TreeControlNode
):
    load_test_tree(tree_control_node)

    expect(open_web_gui.locator("#treeNameForm")).to_have_value(
        "constant_passthrough", timeout=30_000
    )
    for node_name in ("Sequence", "Constant", "Passthrough"):
        expect(
            open_web_gui.locator(".node_name").filter(has_text=node_name)
        ).to_be_visible(timeout=30_000)


def test_ros_state_updates_reach_shipped_ui(
    open_web_gui: Page, tree_control_node: TreeControlNode
):
    load_test_tree(tree_control_node)
    assert tree_control_node.execute_tree(
        ControlTreeExecution.Request.TICK_ONCE
    ).is_ok()

    states = tree_states(tree_control_node)
    assert set(states.values()) == {NodeState.SUCCEEDED}
    expect(open_web_gui.locator(".state-display")).to_contain_text(
        "WAITING_FOR_TICK", timeout=30_000
    )


def test_shipped_ui_commands_tree(
    open_web_gui: Page, tree_control_node: TreeControlNode
):
    load_test_tree(tree_control_node)
    expect(open_web_gui.locator("#treeNameForm")).to_have_value(
        "constant_passthrough", timeout=30_000
    )

    open_web_gui.get_by_title("Tick Once").click()
    expect(open_web_gui.locator(".state-display")).to_contain_text(
        "WAITING_FOR_TICK", timeout=30_000
    )
