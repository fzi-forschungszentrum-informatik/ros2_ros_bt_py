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

from typing import Optional

import rclpy
from rclpy.node import Node
from threading import Lock
import time

from example_interfaces.msg import String

from ros_bt_py_interfaces.srv import ControlTreeExecution

from tests.integration.conftest import TreeControlNode, standard_tree_node


class FooSubscriber(Node):

    def __init__(self, *args, **kwargs) -> None:
        super().__init__("foo_subscriber", *args, **kwargs)

        self._last_message: Optional[String] = None
        self._message_lock = Lock()

        _ = self.create_subscription(
            String,
            "/foo",
            self._callback,
            1,
        )

    def _callback(self, msg: String):
        with self._message_lock:
            self._last_message = msg

    def get_msg(self, wait_time=10) -> Optional[String]:
        start_time = time.time()
        while start_time + wait_time > time.time():
            with self._message_lock:
                if self._last_message is not None:
                    return self._last_message
            rclpy.spin_once(self, timeout_sec=2)
        return None

    def clear_msg(self):
        with self._message_lock:
            self._last_message = None


@pytest.fixture()
def foo_subscriber():
    node = FooSubscriber()
    yield node
    node.destroy_node()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency()
def test_tree_load(tree_control_node: TreeControlNode):
    load_result = tree_control_node.load_tree(
        "trees/ros_nodes_isolation/topic_publish.yaml"
    )
    assert load_result.is_ok()


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_tree_load"])
def test_first_tick_msg_receive(
    tree_control_node: TreeControlNode,
    foo_subscriber: FooSubscriber,
):
    test_tree_load(tree_control_node)

    foo_subscriber.clear_msg()
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    # We expect a message after the first tick
    assert foo_subscriber.get_msg() is not None


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_first_tick_msg_receive"])
def test_second_tick_no_msg_receive(
    tree_control_node: TreeControlNode,
    foo_subscriber: FooSubscriber,
):
    test_first_tick_msg_receive(tree_control_node, foo_subscriber)

    foo_subscriber.clear_msg()
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    # We expect NO message after the second tick
    assert foo_subscriber.get_msg() is None


@pytest.mark.launch(fixture=standard_tree_node)
@pytest.mark.dependency(depends=["test_second_tick_no_msg_receive"])
def test_reset_msg_receive(
    tree_control_node: TreeControlNode,
    foo_subscriber: FooSubscriber,
):
    test_second_tick_no_msg_receive(tree_control_node, foo_subscriber)

    foo_subscriber.clear_msg()
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.RESET)
    assert run_result.is_ok()
    run_result = tree_control_node.execute_tree(ControlTreeExecution.Request.TICK_ONCE)
    assert run_result.is_ok()

    # We expect a message after a reset
    assert foo_subscriber.get_msg() is not None
