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
"""Repeated load → run → shutdown cycles must not leak ROS entities.

This is the regression test for repeated tree execution: a tree is
loaded, run with TICK_UNTIL_RESULT until it reports a result, shut
down, and the next tree is loaded into the same manager. The ROS
entities the tree's nodes created (publishers, subscriptions, timers,
...) must be released by shutdown, so the entity counts on the hosting
node return to their post-construction baseline after every cycle.
"""
import threading
import time
from unittest.mock import MagicMock

import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor

from ros_bt_py.tree_manager import TreeManager
from ros_bt_py_interfaces.msg import TreeState, TreeStructure
from ros_bt_py_interfaces.srv import ControlTreeExecution as Control
from ros_bt_py_interfaces.srv import LoadTree

TREE_PATHS = [
    "package://ros_bt_py/trees/pub_sub_test.yaml",
    "package://ros_bt_py/trees/nested_five_pub_sub_test.yaml",
    "package://ros_bt_py/trees/double_nested_five_pub_sub_test.yaml",
]
CYCLE_TIMEOUT_S = 30.0
ROUNDS = 2


def entity_counts(ros_node):
    """Count the live ROS entities on the hosting node."""
    return {
        "publishers": len(ros_node._publishers),
        "subscriptions": len(ros_node._subscriptions),
        "clients": len(ros_node._clients),
        "services": len(ros_node._services),
        "timers": len(ros_node._timers),
        "guards": len(ros_node._guards),
    }


def run_cycle(manager, tree_path):
    """Run one load → TICK_UNTIL_RESULT → SHUTDOWN cycle, step by step."""
    load_response = manager.load_tree(
        LoadTree.Request(tree=TreeStructure(path=tree_path)),
        LoadTree.Response(),
    )
    assert load_response.success, load_response.error_message

    run_response = manager.control_execution(
        Control.Request(command=Control.Request.TICK_UNTIL_RESULT),
        Control.Response(),
    )
    assert run_response.success, run_response.error_message

    deadline = time.monotonic() + CYCLE_TIMEOUT_S
    while manager.state == TreeState.TICKING and time.monotonic() < deadline:
        time.sleep(0.02)
    assert manager._tick_thread is not None
    manager._tick_thread.join(CYCLE_TIMEOUT_S)
    assert not manager._tick_thread.is_alive()
    assert (
        manager.state == TreeState.IDLE
    ), f"tree did not report a result: state={manager.state}"

    shutdown_response = manager.control_execution(
        Control.Request(command=Control.Request.SHUTDOWN),
        Control.Response(),
    )
    assert shutdown_response.success, shutdown_response.error_message
    assert manager.state == TreeState.EDITABLE


def test_repeated_cycles_do_not_leak_entities():
    """Each cycle must return to the baseline ROS entity counts."""
    context = Context()
    context.init(domain_id=174)
    ros_node = rclpy.create_node(
        "manager_repeated_cycles",
        context=context,
        enable_rosout=False,
    )
    manager = TreeManager(ros_node=ros_node, logging_manager=MagicMock())
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(ros_node)
    spin = threading.Thread(target=executor.spin)
    spin.start()
    try:
        baseline = entity_counts(ros_node)
        for cycle, tree_path in enumerate(TREE_PATHS * ROUNDS):
            run_cycle(manager, tree_path)
            assert entity_counts(ros_node) == baseline, (
                f"cycle {cycle} ({tree_path}) leaked ROS entities: "
                f"{entity_counts(ros_node)} != baseline {baseline}"
            )
    finally:
        executor.shutdown(timeout_sec=3)
        spin.join(3)
        manager.destroy()
        ros_node.destroy_node()
        context.shutdown()
