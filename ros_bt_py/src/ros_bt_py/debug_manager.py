# Copyright 2023 FZI Forschungszentrum Informatik
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
from contextlib import contextmanager
from threading import Lock

import rclpy.node

from ros_bt_py_interfaces.msg import DebugSettings, NodeDiagnostics


class DebugManager(object):
    def __init__(
        self,
        ros_node: rclpy.node.Node,
        debug_settings_publish_callback=None,
        node_diagnostics_publish_callback=None,
    ):
        self._lock = Lock()
        self._ros_node = ros_node

        self.publish_debug_settings = debug_settings_publish_callback
        self.publish_node_diagnostics = node_diagnostics_publish_callback

        self.diagnostics_state = {}
        self.diagnostics_state["SETUP"] = (
            NodeDiagnostics.PRE_SETUP,
            NodeDiagnostics.POST_SETUP,
        )
        self.diagnostics_state["UNTICK"] = (
            NodeDiagnostics.PRE_UNTICK,
            NodeDiagnostics.POST_UNTICK,
        )
        self.diagnostics_state["RESET"] = (
            NodeDiagnostics.PRE_RESET,
            NodeDiagnostics.POST_RESET,
        )
        self.diagnostics_state["SHUTDOWN"] = (
            NodeDiagnostics.PRE_SHUTDOWN,
            NodeDiagnostics.POST_SHUTDOWN,
        )

        self._debug_settings_msg = DebugSettings()

    def set_execution_mode(
        self,
        collect_node_diagnostics,
    ):
        with self._lock:
            self._debug_settings_msg.collect_node_diagnostics = collect_node_diagnostics
        if self.publish_debug_settings:
            self.publish_debug_settings(self._debug_settings_msg)

    @contextmanager
    def report_state(self, node_instance, state):
        """
        Collect debug state from Node execution.

        It measures the time between the beginning and the end of the
        setup/shutdown/reset/untick function (which includes that of any children).

        Additionally, it publishes the node state and execution time
        to a node diagnostics topic.

        :param instance: The node
        :param state: The state of the node
        """
        diagnostics_message = None
        if self._debug_settings_msg.collect_node_diagnostics:
            diagnostics_message = NodeDiagnostics(
                stamp=self._ros_node.get_clock().now(),
                module=type(node_instance).__module__,
                node_class=type(node_instance).__name__,
                name=node_instance.name,
                state=self.diagnostics_state[state][0],
            )
            node = node_instance
            while node is not None:
                diagnostics_message.path.append(node.name)
                node = node.parent
            # reverse the list
            diagnostics_message.path = diagnostics_message.path[::-1]
            if self.publish_node_diagnostics:
                self.publish_node_diagnostics(diagnostics_message)

        # Contextmanager'ed code is executed here
        yield

        if self._debug_settings_msg.collect_node_diagnostics:
            diagnostics_message.state = self.diagnostics_state[state][1]
            diagnostics_message.stamp = self._ros_node.get_clock().now()
            if self.publish_node_diagnostics:
                if self.publish_node_diagnostics:
                    self.publish_node_diagnostics(diagnostics_message)

    @contextmanager
    def report_tick(self, node_instance):
        """
        Collect debug data during ticks from Node execution.

        It measures the time between the beginning and the end of the
        tick function (which includes the ticks of any children) and
        calculates a moving window average of execution times as well as
        a minimum and maximum value.

        Additionally, it provides pause functionality to enable stepping
        through a tree and adding break points.

        :param instance: The node that's executing
        """
        diagnostics_message = None
        if self._debug_settings_msg.collect_node_diagnostics:
            diagnostics_message = NodeDiagnostics(
                stamp=self._ros_node.get_clock().now(),
                module=type(node_instance).__module__,
                node_class=type(node_instance).__name__,
                name=node_instance.name,
                state=NodeDiagnostics.PRE_TICK,
            )
            node = node_instance
            while node is not None:
                diagnostics_message.path.append(node.name)
                node = node.parent
            # reverse the list
            diagnostics_message.path = diagnostics_message.path[::-1]
            if self.publish_node_diagnostics:
                self.publish_node_diagnostics(diagnostics_message)

        # Contextmanager'ed code is executed here
        yield

        if self._debug_settings_msg.collect_node_diagnostics:
            diagnostics_message.state = NodeDiagnostics.POST_TICK
            diagnostics_message.stamp = self._ros_node.get_clock().now()
            if self.publish_node_diagnostics:
                if self.publish_node_diagnostics:
                    self.publish_node_diagnostics(diagnostics_message)
