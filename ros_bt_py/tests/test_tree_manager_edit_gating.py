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

from ros_bt_py.tree_manager import TreeManager


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
        ControlTreeExecution.Request(
            command=ControlTreeExecution.Request.DO_NOTHING
        ),
        ControlTreeExecution.Response(),
    )

    assert acquired_from_another_thread == [False]
