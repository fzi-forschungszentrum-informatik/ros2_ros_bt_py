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
"""Regression tests for the tree state machine around stop/shutdown."""
import threading
import uuid
from unittest.mock import MagicMock

import pytest

from ros_bt_py_interfaces.msg import TreeState
from ros_bt_py_interfaces.srv import ControlTreeExecution

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.vendor.result import Ok


@pytest.fixture
def manager() -> TreeManager:
    return TreeManager(ros_node=MagicMock(), logging_manager=MagicMock())


def make_root(state: str = BTNodeState.IDLE) -> MagicMock:
    root = MagicMock()
    root.node_id = uuid.uuid4()
    root.parent = None
    root.state = state
    return root


def test_shutdown_waits_for_a_tick_thread_that_is_still_unticking(
    manager: TreeManager, monkeypatch
):
    """A SHUTDOWN must join the tick thread before it touches any node.

    tick() publishes IDLE *before* it unticks the tree, so for the whole
    duration of that untick the tree advertises a terminal state while the tick
    thread is still inside it. Deciding on self.state alone lets shutdown() run
    concurrently with untick() on the same nodes.
    """
    monkeypatch.setattr("ros_bt_py.tree_manager.ok", lambda *args, **kwargs: True)

    untick_entered = threading.Event()
    release_untick = threading.Event()
    tick_thread_alive_during_shutdown = []

    def slow_untick():
        untick_entered.set()
        release_untick.wait(timeout=10)
        return Ok(BTNodeState.IDLE)

    def record_shutdown():
        tick_thread_alive_during_shutdown.append(manager._tick_thread.is_alive())
        return Ok(BTNodeState.SHUTDOWN)

    root = make_root()
    root.tick.return_value = Ok(BTNodeState.SUCCEEDED)
    root.untick.side_effect = slow_untick
    root.shutdown.side_effect = record_shutdown
    manager.nodes = {root.node_id: root}

    manager.state = TreeState.TICKING
    manager._stop_after_result = True
    manager._tick_thread = threading.Thread(target=manager.tick_report_exceptions)
    manager._tick_thread.start()

    assert untick_entered.wait(timeout=10)
    # The tree already claims a terminal state even though it is mid-teardown.
    assert manager.state == TreeState.IDLE

    responses = []

    def call_shutdown():
        responses.append(
            manager.control_execution(
                ControlTreeExecution.Request(
                    command=ControlTreeExecution.Request.SHUTDOWN
                ),
                ControlTreeExecution.Response(),
            )
        )

    caller = threading.Thread(target=call_shutdown)
    caller.start()
    try:
        caller.join(timeout=0.5)
        assert caller.is_alive(), "SHUTDOWN returned without joining the tick thread"
        assert tick_thread_alive_during_shutdown == []
    finally:
        release_untick.set()
    caller.join(timeout=10)

    assert not caller.is_alive()
    assert tick_thread_alive_during_shutdown == [False]
    assert responses[0].success
    assert manager.state == TreeState.EDITABLE


def test_stop_on_a_tree_that_is_not_running_reports_success(manager: TreeManager):
    """STOP on an idle/editable tree is a no-op, not a failure."""
    root = make_root()
    manager.nodes = {root.node_id: root}
    assert manager.state == TreeState.EDITABLE

    response = manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.STOP),
        ControlTreeExecution.Response(),
    )

    assert response.success
    assert response.tree_state == TreeState.EDITABLE
    assert response.error_message == ""


def test_stop_while_waiting_for_tick_applies_the_state(manager: TreeManager):
    """The published state must agree with the state reported to the caller."""
    root = make_root()
    root.untick.side_effect = lambda: setattr(root, "state", BTNodeState.IDLE)
    manager.nodes = {root.node_id: root}
    manager.state = TreeState.WAITING_FOR_TICK

    response = manager.control_execution(
        ControlTreeExecution.Request(command=ControlTreeExecution.Request.STOP),
        ControlTreeExecution.Response(),
    )

    assert response.success
    assert response.tree_state == TreeState.IDLE
    assert manager.state == TreeState.IDLE
