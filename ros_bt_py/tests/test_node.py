# Copyright 2026 FZI Forschungszentrum Informatik
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
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
# USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
# THE POSSIBILITY OF SUCH DAMAGE.
"""Regression tests for two approved fixes to the Node base class.

1. ``Node.shutdown`` on a node in state ``UNINITIALIZED`` must not call
   ``_do_shutdown`` and must leave the node in state ``SHUTDOWN``.

   The ``Node.shutdown`` docstring promises: "``_do_shutdown`` will not be
   called if the node has not been initialized yet."

2. Permissive loading in ``Node.from_msg`` must be instance-local, so a
   strict ``Node.from_msg`` call after a permissive one is strict, and a
   permissive load does not change how any other node of the same class is
constructed.
"""
import uuid
from typing import ClassVar, Dict
from unittest.mock import MagicMock, patch

import pytest
import rclpy
from ros_bt_py_interfaces.msg import NodeOption, NodeStructure

from ros_bt_py.exceptions import BehaviorTreeException, NodeConfigError
from ros_bt_py.helpers import BTNodeState, json_encode
from ros_bt_py.node import Leaf, Node, define_bt_node
from ros_bt_py.node_config import NodeConfig
from ros_bt_py.vendor.result import Err, Ok, Result


@pytest.fixture
def ros_node():
    """Provide a real ROS node; ``from_msg`` is type-checked and needs one."""
    we_initialized = not rclpy.ok()
    if we_initialized:
        rclpy.init()
    node = rclpy.create_node("test_node_regressions")
    yield node
    node.destroy_node()
    if we_initialized:
        rclpy.shutdown()


def make_structure(node_class, options, node_id=None):
    """Build a NodeStructure message for the given class and options."""
    return NodeStructure(
        module=node_class.__module__,
        node_class=node_class.__name__,
        node_id=node_id or str(uuid.uuid4()),
        options=[
            NodeOption(key=key, serialized_value=json_encode(value))
            for key, value in options.items()
        ],
    )


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class ShutdownSpy(Leaf):
    """Leaf that counts ``_do_shutdown`` calls and rejects them while UNINITIALIZED."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.do_shutdown_calls = 0

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.do_shutdown_calls += 1
        if self.state == BTNodeState.UNINITIALIZED:
            return Err(
                BehaviorTreeException(
                    "_do_shutdown must not be called on an uninitialized node"
                )
            )
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"probe_type": type},
        inputs={},
        outputs={},
        max_children=0,
    )
)
class PermissiveProbe(Leaf):
    """Leaf with a single ``type`` option to probe permissive loading.

    The option data type is exactly ``type``, which is the case the
    permissive branch in ``Node._find_option_refs`` coerces to ``int``.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)


class TestShutdownOfUninitializedNode:
    """Node.shutdown on an UNINITIALIZED node must skip _do_shutdown and yield SHUTDOWN."""

    def test_results_in_shutdown(self):
        node = ShutdownSpy()
        assert node.state == BTNodeState.UNINITIALIZED

        result = node.shutdown()

        assert result.is_ok()
        assert result.unwrap() == BTNodeState.SHUTDOWN
        assert node.state == BTNodeState.SHUTDOWN

    def test_skips_do_shutdown(self):
        node = ShutdownSpy()
        node.shutdown()
        assert node.do_shutdown_calls == 0

    def test_initialized_node_still_calls_do_shutdown(self):
        """Guard: the fix must not skip _do_shutdown for initialized nodes."""
        node = ShutdownSpy()
        assert node.setup().unwrap() == BTNodeState.IDLE

        result = node.shutdown()

        assert result.is_ok()
        assert node.do_shutdown_calls == 1
        assert node.state == BTNodeState.SHUTDOWN

    def test_repeated_shutdown_does_not_revisit_descendants(self):
        parent = ShutdownSpy()
        child = ShutdownSpy()
        parent.children.append(child)
        child.parent = parent
        parent.setup()
        child.setup()

        parent.shutdown()
        with patch.object(child, "shutdown", wraps=child.shutdown) as shutdown:
            parent.shutdown()

        assert parent.do_shutdown_calls == 1
        assert child.do_shutdown_calls == 1
        shutdown.assert_not_called()

    def test_repeated_shutdown_revisits_a_child_left_broken(self):
        """A child whose own shutdown failed must be revisited on retry, not skipped."""
        parent = ShutdownSpy()
        child = MagicMock()
        child.state = BTNodeState.UNINITIALIZED
        child.name = "child"

        def fail_once():
            if child.shutdown.call_count == 1:
                child.state = BTNodeState.BROKEN
                return Err(BehaviorTreeException("boom"))
            child.state = BTNodeState.SHUTDOWN
            return Ok(BTNodeState.SHUTDOWN)

        child.shutdown.side_effect = fail_once
        parent.children.append(child)
        parent.setup()

        first_result = parent.shutdown()
        assert first_result.is_err()
        assert parent.state == BTNodeState.SHUTDOWN
        assert child.state == BTNodeState.BROKEN

        second_result = parent.shutdown()

        assert child.shutdown.call_count == 2
        assert second_result.is_ok()
        assert child.state == BTNodeState.SHUTDOWN
        assert parent.do_shutdown_calls == 1, "parent must not be re-shut-down"


class TestPermissiveLoadingIsInstanceLocal:
    """A permissive ``Node.from_msg`` call must not affect any other construction."""

    BAD_OPTIONS: ClassVar[Dict[str, str]] = {"probe_type": "not-a-type"}

    @pytest.fixture(autouse=True)
    def reset_leak(self):
        """Isolate tests from the permissive class attribute leak of the current code."""
        PermissiveProbe.permissive = False
        yield
        PermissiveProbe.permissive = False

    def test_permissive_from_msg_does_not_mutate_node_class(self, ros_node):
        permissive_result = Node.from_msg(
            make_structure(PermissiveProbe, self.BAD_OPTIONS),
            ros_node=ros_node,
            permissive=True,
        )
        assert permissive_result.is_ok()
        assert (
            permissive_result.unwrap().permissive is True
        ), "the permissively loaded instance itself must be permissive"
        assert (
            PermissiveProbe.permissive is False
        ), "a permissive from_msg must not set permissive on the node class"

    def test_strict_from_msg_after_permissive_from_msg_is_strict(self, ros_node):
        bad_msg = make_structure(PermissiveProbe, self.BAD_OPTIONS)
        permissive_result = Node.from_msg(
            bad_msg, ros_node=ros_node, permissive=True
        )
        assert permissive_result.is_ok()

        strict_result = Node.from_msg(bad_msg, ros_node=ros_node, permissive=False)

        assert strict_result.is_err()

    def test_direct_instantiation_after_permissive_from_msg_is_strict(self, ros_node):
        permissive_result = Node.from_msg(
            make_structure(PermissiveProbe, self.BAD_OPTIONS),
            ros_node=ros_node,
            permissive=True,
        )
        assert permissive_result.is_ok()

        with pytest.raises(NodeConfigError):
            PermissiveProbe(options=self.BAD_OPTIONS)
