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
import pytest
import unittest.mock as mock

from copy import deepcopy
from result import Result, Ok, Err

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Node
from ros_bt_py.nodes.fallback import Fallback, MemoryFallback, NameSwitch


@pytest.fixture
def mock_node() -> mock.NonCallableMagicMock:
    mock_node = mock.NonCallableMagicMock(set=Node)
    mock_node.setup.return_value = Ok(BTNodeState.IDLE)
    mock_node.tick.return_value = Ok(BTNodeState.FAILED)
    mock_node.untick.return_value = Ok(BTNodeState.IDLE)
    mock_node.reset.return_value = Ok(BTNodeState.IDLE)
    mock_node.shutdown.return_value = Ok(BTNodeState.SHUTDOWN)
    return mock_node


class TestFallback:

    @pytest.fixture
    def target_node(self) -> Fallback:
        node = Fallback()
        node.state = BTNodeState.IDLE
        return node

    def test_setup(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid additional checks
        target_node.children = mock_children  # type: ignore
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.setup()]

    def test_tick_success(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.SUCCEEDED)
        mock_child_3 = deepcopy(mock_node)
        # We add node children directly to avoid additional checks
        target_node.children = [mock_child_1, mock_child_2, mock_child_3]

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_child_1.mock_calls == [mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == [mock.call.untick()]

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()
        mock_child_3.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_child_1.mock_calls == [mock.call.reset(), mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.reset(), mock.call.tick()]
        assert mock_child_3.mock_calls == [mock.call.reset(), mock.call.untick()]

    def test_tick_running(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.RUNNING)
        mock_child_3 = deepcopy(mock_node)

        # We add node children directly to avoid node additional checks
        target_node.children = [mock_child_1, mock_child_2, mock_child_3]

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_child_1.mock_calls == [mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == []

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()
        mock_child_3.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_child_1.mock_calls == [mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == []

    def test_tick_failure(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.tick()]

        for m_c in mock_children:
            m_c.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.reset(), mock.call.tick()]

    def test_untick(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.untick()]

    def test_reset(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.reset()]


class TestMemoryFallback:

    @pytest.fixture
    def target_node(self) -> MemoryFallback:
        node = MemoryFallback()
        node.state = BTNodeState.IDLE
        node.last_running_child = -1
        return node

    def test_setup(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid additional checks
        target_node.children = mock_children  # type: ignore
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.setup()]

    def test_tick_success(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.SUCCEEDED)
        mock_child_3 = deepcopy(mock_node)
        # We add node children directly to avoid additional checks
        target_node.children = [mock_child_1, mock_child_2, mock_child_3]

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.last_running_child == -1

        assert mock_child_1.mock_calls == [mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == [mock.call.untick()]

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()
        mock_child_3.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.last_running_child == 0

        assert mock_child_1.mock_calls == [mock.call.reset(), mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.reset(), mock.call.tick()]
        assert mock_child_3.mock_calls == [mock.call.reset(), mock.call.untick()]

    def test_tick_running(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.RUNNING)
        mock_child_3 = deepcopy(mock_node)

        # We add node children directly to avoid node additional checks
        target_node.children = [mock_child_1, mock_child_2, mock_child_3]

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING
        assert target_node.last_running_child == 1

        assert mock_child_1.mock_calls == [mock.call.tick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == []

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()
        mock_child_3.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING
        assert target_node.last_running_child == 1

        assert mock_child_1.mock_calls == []
        assert mock_child_2.mock_calls == [mock.call.tick()]
        assert mock_child_3.mock_calls == []

    def test_tick_failure(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED
        assert target_node.last_running_child == -1

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.tick()]

        for m_c in mock_children:
            m_c.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED
        assert target_node.last_running_child == 0

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.reset(), mock.call.tick()]

    def test_untick(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.untick()]

    def test_reset(
        self,
        target_node: MemoryFallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.reset()]


class TestNameSwitch:

    @pytest.fixture
    def target_node(self) -> NameSwitch:
        node = NameSwitch()
        node.state = BTNodeState.IDLE
        return node

    def test_setup(
        self,
        target_node: NameSwitch,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        for i, m_c in enumerate(mock_children):
            m_c.name = f"child{i}"
        # We add node children directly to avoid additional checks
        target_node.children = mock_children  # type: ignore
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.setup()]

    def test_tick_success(
        self,
        target_node: NameSwitch,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.SUCCEEDED)
        # We add node children directly to avoid additional checks
        target_node.children = [mock_child_1, mock_child_2]
        target_node.child_map = {"child1": mock_child_1, "child2": mock_child_2}  # type: ignore
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_child_1.mock_calls == [mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_child_1.mock_calls == [mock.call.reset(), mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.reset(), mock.call.tick()]

    def test_tick_running(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        mock_child_2.tick.return_value = Ok(BTNodeState.RUNNING)
        # We add node children directly to avoid additional checks
        target_node.children = [mock_child_1, mock_child_2]
        target_node.child_map = {"child1": mock_child_1, "child2": mock_child_2}  # type: ignore
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_child_1.mock_calls == [mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_child_1.mock_calls == [mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]

    def test_tick_failure(
        self,
        target_node: Fallback,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_child_1 = deepcopy(mock_node)
        mock_child_2 = deepcopy(mock_node)
        # We add node children directly to avoid additional checks
        target_node.children = [mock_child_1, mock_child_2]
        target_node.child_map = {"child1": mock_child_1, "child2": mock_child_2}  # type: ignore
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_child_1.mock_calls == [mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.tick()]

        mock_child_1.reset_mock()
        mock_child_2.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_child_1.mock_calls == [mock.call.reset(), mock.call.untick()]
        assert mock_child_2.mock_calls == [mock.call.reset(), mock.call.tick()]

    def test_untick(
        self,
        target_node: NameSwitch,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.untick()]

    def test_reset(
        self,
        target_node: NameSwitch,
        mock_node: mock.NonCallableMagicMock,
    ):
        mock_children = [deepcopy(mock_node) for _ in range(3)]
        # We add node children directly to avoid node additional checks
        target_node.children = mock_children  # type: ignore

        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        for m_c in mock_children:
            assert m_c.mock_calls == [mock.call.reset()]
