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

from threading import RLock
from unittest.mock import MagicMock

from ros_bt_py.nodes.sequence import MemorySequence
from ros_bt_py.ros_helpers import uuid_to_ros
from ros_bt_py.tree_edit_manager import TreeEditManager
from ros_bt_py_interfaces.msg import TreeState, TreeStructure
from ros_bt_py_interfaces.srv import RemoveNode


def make_manager(nodes, children):
    manager = TreeEditManager.__new__(TreeEditManager)
    manager._edit_lock = RLock()
    manager._tree_state = TreeState(state=TreeState.EDITABLE)
    manager._tree_structure = TreeStructure(name="broken")
    manager._tree_structure.data_wirings = []
    manager.nodes = {node.node_id: node for node in nodes}
    manager._children = children
    manager.subtree_manager = MagicMock()
    manager.logging_manager = MagicMock()
    manager.publish_structure = MagicMock()
    return manager


def test_remove_node_repairs_a_forest():
    first = MemorySequence()
    second = MemorySequence()
    manager = make_manager([first, second], {first.node_id: [], second.node_id: []})

    response = manager.remove_node(
        RemoveNode.Request(node_id=uuid_to_ros(first.node_id), remove_children=False),
        RemoveNode.Response(),
    )

    assert response.success
    assert manager.nodes == {second.node_id: second}
    assert manager._children == {second.node_id: []}


def test_remove_node_orphans_surviving_children():
    parent = MemorySequence()
    child = MemorySequence()
    parent.add_child(child)
    manager = make_manager(
        [parent, child], {parent.node_id: [child.node_id], child.node_id: []}
    )

    response = manager.remove_node(
        RemoveNode.Request(node_id=uuid_to_ros(parent.node_id), remove_children=False),
        RemoveNode.Response(),
    )

    assert response.success
    assert manager.nodes == {child.node_id: child}
    assert child.parent is None
    assert manager._children == {child.node_id: []}


def test_find_nodes_in_cycles_ignores_dangling_parent():
    node = MemorySequence()
    deleted_parent = MemorySequence()
    node.parent = deleted_parent
    manager = make_manager([node], {node.node_id: []})

    assert manager.find_nodes_in_cycles() == []


def test_remove_children_terminates_for_a_cycle():
    first = MemorySequence()
    second = MemorySequence()
    manager = make_manager(
        [first, second],
        {first.node_id: [second.node_id], second.node_id: [first.node_id]},
    )
    first.children = [second]
    second.children = [first]

    response = manager.remove_node(
        RemoveNode.Request(node_id=uuid_to_ros(first.node_id), remove_children=True),
        RemoveNode.Response(),
    )

    assert response.success
    assert manager.nodes == {}
    assert manager._children == {}
