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
from ros_bt_py.vendor.result import Result, Ok, Err

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.nodes.fallback import Fallback, MemoryFallback, NameSwitch


@pytest.fixture
def mock_obj() -> mock.NonCallableMagicMock:
    mock_obj = mock.NonCallableMagicMock()
    mock_obj.children = []
    for i in range(1, 4):
        mock_node = mock.NonCallableMagicMock(
            spec_set=["name", "setup", "tick", "untick", "reset", "shutdown"]
        )
        mock_node.setup.return_value = Ok(BTNodeState.IDLE)
        mock_node.tick.return_value = Ok(BTNodeState.FAILED)
        mock_node.untick.return_value = Ok(BTNodeState.IDLE)
        mock_node.reset.return_value = Ok(BTNodeState.IDLE)
        mock_node.shutdown.return_value = Ok(BTNodeState.SHUTDOWN)
        setattr(mock_obj, f"child{i}", mock_node)
        mock_obj.children.append(mock_node)
    return mock_obj


class TestFallback:

    @pytest.fixture
    def target_node(self, mock_obj, logging_mock) -> Fallback:
        node = Fallback(logging_manager=logging_mock)
        # We add node children directly to avoid node additional checks
        node.children = mock_obj.children
        node.state = BTNodeState.IDLE
        return node

    def test_setup(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        assert mock_obj.method_calls == [
            mock.call.child1.setup(),
            mock.call.child2.setup(),
            mock.call.child3.setup(),
        ]

    def test_tick_success(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.SUCCEEDED)

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.untick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.untick(),
        ]

    def test_tick_running(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.RUNNING)

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
        ]

    def test_tick_failure(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.tick(),
        ]

    def test_untick(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child2.untick(),
            mock.call.child3.untick(),
        ]

    def test_reset(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
        ]


class TestMemoryFallback:

    @pytest.fixture
    def target_node(self, mock_obj, logging_mock) -> MemoryFallback:
        node = MemoryFallback(logging_manager=logging_mock)
        # We add node children directly to avoid node additional checks
        node.children = mock_obj.children
        node.state = BTNodeState.IDLE
        node.last_running_child = -1
        return node

    def test_setup(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        assert mock_obj.method_calls == [
            mock.call.child1.setup(),
            mock.call.child2.setup(),
            mock.call.child3.setup(),
        ]

    def test_tick_success(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.SUCCEEDED)

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.last_running_child == -1

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.untick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED
        assert target_node.last_running_child == 0

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.untick(),
        ]

    def test_tick_running(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.RUNNING)

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING
        assert target_node.last_running_child == 1

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING
        assert target_node.last_running_child == 1

        assert mock_obj.method_calls == [
            mock.call.child2.tick(),
        ]

    def test_tick_failure(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED
        assert target_node.last_running_child == -1

        assert mock_obj.method_calls == [
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED
        assert target_node.last_running_child == 0

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.tick(),
            mock.call.child2.tick(),
            mock.call.child3.tick(),
        ]

    def test_untick(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child2.untick(),
            mock.call.child3.untick(),
        ]

    def test_reset(
        self,
        target_node: MemoryFallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.last_running_child == 0

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
        ]


class TestNameSwitch:

    @pytest.fixture
    def target_node(self, mock_obj, logging_mock) -> NameSwitch:
        node = NameSwitch(logging_manager=logging_mock)
        # We add node children directly to avoid node additional checks
        node.children = mock_obj.children
        node.child_map = {
            f"child{i + 1}": child for i, child in enumerate(mock_obj.children)
        }
        node.state = BTNodeState.IDLE
        return node

    def test_setup(
        self,
        target_node: NameSwitch,
        mock_obj: mock.NonCallableMagicMock,
    ):
        for i, m_c in enumerate(mock_obj.children):
            m_c.name = f"child{i}"
        # Node has to be shutdown to call setup
        target_node.state = BTNodeState.SHUTDOWN
        target_node.child_map = {}

        result = target_node.setup()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE
        assert target_node.child_map == {
            f"child{i}": m_c for i, m_c in enumerate(mock_obj.children)
        }

        assert mock_obj.method_calls == [
            mock.call.child1.setup(),
            mock.call.child2.setup(),
            mock.call.child3.setup(),
        ]

    def test_tick_success(
        self,
        target_node: NameSwitch,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.SUCCEEDED)
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SUCCEEDED
        assert target_node.state == BTNodeState.SUCCEEDED

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

    def test_tick_running(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        mock_obj.child2.tick.return_value = Ok(BTNodeState.RUNNING)
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should NOT involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.RUNNING
        assert target_node.state == BTNodeState.RUNNING

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

    def test_tick_failure(
        self,
        target_node: Fallback,
        mock_obj: mock.NonCallableMagicMock,
    ):
        target_node.inputs["name"] = "child2"

        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

        mock_obj.reset_mock()

        # The second tick should also involve a reset on all child nodes
        result = target_node.tick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.FAILED
        assert target_node.state == BTNodeState.FAILED

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
            mock.call.child1.untick(),
            mock.call.child3.untick(),
            mock.call.child2.tick(),
        ]

    def test_untick(
        self,
        target_node: NameSwitch,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.untick()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        assert mock_obj.method_calls == [
            mock.call.child1.untick(),
            mock.call.child2.untick(),
            mock.call.child3.untick(),
        ]

    def test_reset(
        self,
        target_node: NameSwitch,
        mock_obj: mock.NonCallableMagicMock,
    ):
        result = target_node.reset()
        assert result.is_ok()
        assert result.unwrap() == BTNodeState.IDLE
        assert target_node.state == BTNodeState.IDLE

        assert mock_obj.method_calls == [
            mock.call.child1.reset(),
            mock.call.child2.reset(),
            mock.call.child3.reset(),
        ]
