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

from threading import Lock, RLock, Thread
from unittest.mock import MagicMock

from ros_bt_py.nodes.sequence import MemorySequence
from ros_bt_py.tree_exec_manager import TreeExecManager
from ros_bt_py_interfaces.msg import TreeState, TreeStructure
from ros_bt_py_interfaces.srv import ClearTree


def test_clear_nonempty_tree_does_not_deadlock():
    manager = TreeExecManager.__new__(TreeExecManager)
    manager._tree_lock = Lock()
    manager._edit_lock = RLock()
    manager._tree_structure = TreeStructure(name="loaded tree", path="file://tree.yaml")
    manager._tree_structure.data_wirings = []
    manager._tree_state = TreeState(state=TreeState.EDITABLE)
    root = MemorySequence()
    manager.nodes = {root.node_id: root}
    manager.logging_manager = MagicMock()
    manager.subtree_manager = MagicMock()
    manager.clear_diagnostics_name = MagicMock()
    manager.publish_structure = MagicMock()

    responses = []
    clear_thread = Thread(
        target=lambda: responses.append(manager.clear(None, ClearTree.Response())),
        daemon=True,
    )
    clear_thread.start()
    clear_thread.join(timeout=1.0)

    assert not clear_thread.is_alive(), "clearing a nonempty tree deadlocked"
    assert responses[0].success
    assert manager.nodes == {}
    assert manager.name == "UNKNOWN TREE"
    assert manager._tree_structure.path == ""
    manager.logging_manager.set_tree_name.assert_called_once_with("UNKNOWN TREE")
