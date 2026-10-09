# Copyright 2025 FZI Forschungszentrum Informatik
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
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
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
"""Regression tests: edits must not run while the tree is executing."""
import threading
import time
import uuid
from unittest.mock import MagicMock

import pytest

from ros_bt_py_interfaces.msg import TreeState, TreeStructure
from ros_bt_py_interfaces.srv import (
    ClearTree,
    ControlTreeExecution,
    LoadTree,
    MigrateTree,
)

from ros_bt_py.tree_manager import TreeManager, validate_tree_topology
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.nodes.sequence import Sequence
from ros_bt_py.vendor.result import Err, Ok


@pytest.fixture
def manager() -> TreeManager:
    return TreeManager(ros_node=MagicMock(), logging_manager=MagicMock())


@pytest.fixture
def running_tick_thread(manager: TreeManager):
    """Keep a tick thread alive for the duration of a test."""
    release = threading.Event()
    manager._tick_thread = threading.Thread(target=release.wait)
    manager._tick_thread.start()
    yield
    release.set()
    manager._tick_thread.join(timeout=5)


@pytest.mark.usefixtures("running_tick_thread")
def test_load_tree_is_rejected_while_the_tick_thread_is_alive(manager: TreeManager):
    """The tree advertises EDITABLE, but the tick thread is still in it.

    That is exactly the window a queued load used to slip through, rebuilding
    self.nodes with fresh (UNINITIALIZED) nodes under the running tree.
    """
    manager.state = TreeState.EDITABLE

    response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not response.success
    assert "still running" in response.error_message


@pytest.mark.usefixtures("running_tick_thread")
def test_clear_is_rejected_while_the_tick_thread_is_alive(manager: TreeManager):
    manager.state = TreeState.EDITABLE

    response = manager.clear(None, ClearTree.Response())

    assert not response.success
    assert "still running" in response.error_message


def test_load_tree_aborts_when_the_tree_cannot_be_cleared(
    manager: TreeManager, monkeypatch
):
    """load_tree used to discard clear()'s response and load anyway.

    The new nodes then ended up alongside the old ones instead of replacing
    them.
    """
    migrate_response = MigrateTree.Response()
    migrate_response.success = True
    migrate_response.tree = TreeStructure()
    monkeypatch.setattr(
        "ros_bt_py.tree_manager.load_tree_from_file",
        lambda request, response: migrate_response,
    )

    def failing_clear(request, response):
        response.success = False
        response.error_message = "Please shut down the tree before clearing it"
        return response

    monkeypatch.setattr(manager, "clear", failing_clear)

    response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not response.success
    assert "Please shut down the tree before clearing it" in response.error_message


def test_stop_still_reaches_a_tree_that_ticks_forever(
    manager: TreeManager, monkeypatch
):
    """Holding the edit lock must not lock the user out of their own STOP.

    A TICK_PERIODICALLY tree never stops on its own, so STOP is the only way
    out. It has to get through the lock and join the tick thread.
    """
    monkeypatch.setattr("ros_bt_py.tree_manager.ok", lambda *args, **kwargs: True)

    ticking = threading.Event()

    def tick_until_stop_requested():
        ticking.set()
        while manager.state != TreeState.STOP_REQUESTED:
            time.sleep(0.005)
        manager.state = TreeState.IDLE

    monkeypatch.setattr(manager, "tick_report_exceptions", tick_until_stop_requested)

    root = MagicMock()
    root.node_id = uuid.uuid4()
    root.parent = None
    manager.nodes = {root.node_id: root}

    start = manager.control_execution(
        ControlTreeExecution.Request(
            command=ControlTreeExecution.Request.TICK_PERIODICALLY
        ),
        ControlTreeExecution.Response(),
    )
    assert start.success
    assert ticking.wait(timeout=5), "tick thread never started"

    stop = manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.STOP),
        ControlTreeExecution.Response(),
    )

    assert stop.success, stop.error_message
    assert stop.tree_state == TreeState.IDLE
    assert not manager._tick_thread.is_alive()


def test_load_tree_is_rejected_after_destroy(manager: TreeManager):
    """A manager that has released its resources must reject further edits."""
    manager.destroy()

    response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not response.success
    assert "destroyed" in response.error_message


def test_control_execution_is_rejected_after_destroy(manager: TreeManager):
    """A manager that has released its resources must reject control commands."""
    manager.destroy()

    response = manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.DO_NOTHING),
        ControlTreeExecution.Response(),
    )

    assert not response.success
    assert "destroyed" in response.error_message


def test_control_execution_holds_the_edit_lock(manager: TreeManager):
    """No edit service can get in while a control command is being handled."""
    acquired_from_another_thread = []

    def check_lock(request, response):
        probe = threading.Thread(
            target=lambda: acquired_from_another_thread.append(
                manager._edit_lock.acquire(blocking=False)
            )
        )
        probe.start()
        probe.join(timeout=5)
        return response

    manager._control_execution = check_lock

    manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.DO_NOTHING),
        ControlTreeExecution.Response(),
    )

    assert acquired_from_another_thread == [False]


def migrate_to(tree):
    response = MigrateTree.Response(success=True)
    response.tree = tree
    return response


def test_invalid_topology_is_rejected_before_the_current_tree_is_cleared(
    manager: TreeManager, monkeypatch
):
    current_root = MagicMock()
    current_root.node_id = uuid.uuid4()
    current_root.parent = None
    current_root.state = BTNodeState.UNINITIALIZED
    manager.nodes = {current_root.node_id: current_root}
    invalid_tree = TreeStructure(
        nodes=[
            Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg(),
            Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg(),
        ]
    )
    monkeypatch.setattr(
        "ros_bt_py.tree_manager.load_tree_from_file",
        lambda request, response: migrate_to(invalid_tree),
    )

    response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not response.success
    assert "root" in response.error_message.lower()
    assert manager.nodes == {current_root.node_id: current_root}


@pytest.mark.parametrize("invalid_kind", ["duplicate", "missing_child", "cycle"])
def test_tree_topology_validation_rejects_malformed_messages(invalid_kind):
    first = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    second = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    if invalid_kind == "duplicate":
        second.node_id = first.node_id
    elif invalid_kind == "missing_child":
        first.child_ids = [str(uuid.uuid4())]
    else:
        first.child_ids = [second.node_id]
        second.child_ids = [first.node_id]

    result = validate_tree_topology(TreeStructure(nodes=[first, second]))

    assert result.is_err()


def test_failed_partial_load_cleans_constructed_nodes(
    manager: TreeManager, monkeypatch
):
    root_msg = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    child_msg = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    root_msg.child_ids = [child_msg.node_id]
    tree = TreeStructure(nodes=[root_msg, child_msg])
    monkeypatch.setattr(
        "ros_bt_py.tree_manager.load_tree_from_file",
        lambda request, response: migrate_to(tree),
    )
    constructed = MagicMock()
    constructed.node_id = uuid.UUID(root_msg.node_id)
    constructed.parent = None
    constructed.shutdown.return_value = Ok(BTNodeState.SHUTDOWN)
    manager.instantiate_node_from_msg = MagicMock(
        side_effect=[Ok(constructed), Err(BehaviorTreeException("construction failed"))]
    )

    response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not response.success
    assert "construction failed" in response.error_message
    constructed.shutdown.assert_called_once()
    assert manager.nodes == {}
    assert manager.state == TreeState.EDITABLE


def test_failed_load_cleanup_can_be_retried_by_shutdown(
    manager: TreeManager, monkeypatch
):
    root_msg = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    child_msg = Sequence(node_id=uuid.uuid4(), ros_node=MagicMock()).to_structure_msg()
    root_msg.child_ids = [child_msg.node_id]
    tree = TreeStructure(nodes=[root_msg, child_msg])
    monkeypatch.setattr(
        "ros_bt_py.tree_manager.load_tree_from_file",
        lambda request, response: migrate_to(tree),
    )
    constructed = MagicMock()
    constructed.node_id = uuid.UUID(root_msg.node_id)
    constructed.parent = None
    constructed.shutdown.side_effect = [
        Err(BehaviorTreeException("cleanup pending")),
        Ok(BTNodeState.SHUTDOWN),
    ]
    manager.instantiate_node_from_msg = MagicMock(
        side_effect=[Ok(constructed), Err(BehaviorTreeException("construction failed"))]
    )

    load_response = manager.load_tree(LoadTree.Request(), LoadTree.Response())

    assert not load_response.success
    assert "cleanup pending" in load_response.error_message
    assert manager.state == TreeState.ERROR
    shutdown_response = manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.SHUTDOWN),
        ControlTreeExecution.Response(),
    )
    assert shutdown_response.success
    assert manager.state == TreeState.EDITABLE
    assert constructed.shutdown.call_count == 2
