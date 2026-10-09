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
"""Exercise manager shutdown against real ROS timers, not mocked Rate.sleep()."""
import threading
import uuid
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from ros_bt_py.helpers import BTNodeState
from ros_bt_py.tree_manager import TreeManager
from ros_bt_py.vendor.result import Ok
from ros_bt_py_interfaces.msg import TreeState
from ros_bt_py_interfaces.srv import ControlTreeExecution as Control


@pytest.mark.parametrize("destroy", [False, True])
def test_shutdown_wakes_a_real_rate_with_paused_ros_time(destroy):
    context = Context()
    context.init(domain_id=173)
    ros_node = rclpy.create_node(
        "manager_paused_clock",
        context=context,
        enable_rosout=False,
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    manager = TreeManager(ros_node=ros_node, logging_manager=MagicMock())
    executor = MultiThreadedExecutor(num_threads=3, context=context)
    executor.add_node(ros_node)
    spin = None
    caller = None
    rate = manager.rate
    sleeping = threading.Event()
    original_sleep = rate.sleep

    def sleep():
        sleeping.set()
        original_sleep()

    rate.sleep = sleep
    root = MagicMock()
    root.node_id = uuid.uuid4()
    root.parent = None
    root.state = BTNodeState.IDLE
    root.tick.return_value = Ok(BTNodeState.RUNNING)
    root.untick.return_value = Ok(BTNodeState.IDLE)
    root.shutdown.return_value = Ok(BTNodeState.SHUTDOWN)
    manager.nodes = {root.node_id: root}
    responses = []
    try:
        if not destroy:
            spin = threading.Thread(target=executor.spin)
            spin.start()
        response = manager.control_execution(
            Control.Request(command=Control.Request.TICK_UNTIL_RESULT),
            Control.Response(),
        )
        assert response.success
        assert sleeping.wait(2)

        def shutdown():
            if destroy:
                responses.append(manager.destroy().is_ok())
            else:
                responses.append(
                    manager.control_execution(
                        Control.Request(command=Control.Request.SHUTDOWN),
                        Control.Response(),
                    )
                )

        caller = threading.Thread(target=shutdown)
        caller.start()
        caller.join(1)
        assert not caller.is_alive(), "shutdown is waiting for the paused ROS clock"
        if destroy:
            assert responses == [True]
        else:
            assert responses[0].success, responses[0].error_message
        assert not manager._tick_thread.is_alive()
        if not destroy:
            assert manager.state == TreeState.EDITABLE
            root.tick.return_value = Ok(BTNodeState.SUCCEEDED)
            response = manager.control_execution(
                Control.Request(command=Control.Request.TICK_UNTIL_RESULT),
                Control.Response(),
            )
            assert response.success
            manager._tick_thread.join(2)
            assert not manager._tick_thread.is_alive()
            assert manager.state == TreeState.IDLE
    finally:
        # Release the original implementation too, so RED never hangs pytest.
        rate.destroy()
        if caller is not None:
            caller.join(3)
        if manager._tick_thread is not None:
            manager._tick_thread.join(3)
        manager.destroy()
        executor.shutdown(timeout_sec=3)
        if spin is not None:
            spin.join(3)
        ros_node.destroy_node()
        context.shutdown()
