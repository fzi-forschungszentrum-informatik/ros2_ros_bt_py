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
from copy import deepcopy
from contextlib import contextmanager
import inspect
from sys import getrecursionlimit
from threading import Event, Lock

import rclpy.node

from ros_bt_py_interfaces.msg import DebugInfo, DebugSettings, Node, NodeDiagnostics


class DebugManager(object):
    def __init__(
        self,
        ros_node: rclpy.node.Node,
        debug_info_publish_callback=None,
        debug_settings_publish_callback=None,
        node_diagnostics_publish_callback=None,
    ):
        self.continue_event = Event()
        self._lock = Lock()
        self._ros_node = ros_node

        self.publish_debug_info = debug_info_publish_callback
        self.publish_debug_settings = debug_settings_publish_callback
        self.publish_node_diagnostics = node_diagnostics_publish_callback

        self.subtrees = {}

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

        with self._lock:
            self._debug_info_msg = DebugInfo()

        self._debug_settings_msg = DebugSettings(
            # List of node names to break on
            breakpoint_names=[],
            # Only collect analytics if this is True
            collect_performance_data=False,
            # Don't publish subtree states by default
            publish_subtrees=False,
            # if True, wait for a continue request before and after every tick
            single_step=False,
        )

    def set_execution_mode(
        self,
        single_step,
        collect_performance_data,
        publish_subtrees,
        collect_node_diagnostics,
    ):
        was_debugging = self.is_debugging()
        with self._lock:
            self._debug_settings_msg.single_step = single_step
            self._debug_settings_msg.collect_performance_data = collect_performance_data
            self._debug_settings_msg.publish_subtrees = publish_subtrees
            self._debug_settings_msg.collect_node_diagnostics = collect_node_diagnostics
        if was_debugging and not self.is_debugging():
            # stopped debugging in this call, send a continue event to prevent issues
            # with control_execution in the tree_manager
            self.continue_debug()
        if self.publish_debug_settings:
            self.publish_debug_settings(self._debug_settings_msg)

    def modify_breakpoints(self, add=None, remove=None, remove_all=False):
        with self._lock:
            if remove_all:
                self._debug_settings_msg.breakpoint_names = []
            if remove:
                for bp in remove:
                    if bp in self._debug_settings_msg.breakpoint_names:
                        self._debug_settings_msg.breakpoint_names.remove(bp)
            if add:
                for bp in add:
                    if bp not in self._debug_settings_msg.breakpoint_names:
                        self._debug_settings_msg.breakpoint_names.append(bp)
            if self.publish_debug_settings:
                self.publish_debug_settings(self._debug_settings_msg)
            return self._debug_settings_msg.breakpoint_names

    def continue_debug(self):
        self.continue_event.set()

    def is_debugging(self):
        with self._lock:
            return (
                self._debug_settings_msg.breakpoint_names
                or self._debug_settings_msg.single_step
            )

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
        if self.is_debugging():
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_PRE_TICK
            self.wait_for_continue()
            node_instance.state = old_state
        if self._debug_settings_msg.collect_performance_data:
            with self._lock:
                self._debug_info_msg.current_recursion_depth = len(inspect.stack())
                self._debug_info_msg.max_recursion_depth = getrecursionlimit()

        # Contextmanager'ed code is executed here
        yield

        if self._debug_settings_msg.collect_performance_data:
            with self._lock:
                self._debug_info_msg.current_recursion_depth = len(inspect.stack())
                self._debug_info_msg.max_recursion_depth = getrecursionlimit()

        if self._debug_settings_msg.collect_node_diagnostics:
            diagnostics_message.state = NodeDiagnostics.POST_TICK
            diagnostics_message.stamp = self._ros_node.get_clock().now()
            if self.publish_node_diagnostics:
                if self.publish_node_diagnostics:
                    self.publish_node_diagnostics(diagnostics_message)
        if self.is_debugging():
            self.wait_for_continue()
            old_state = node_instance.state
            node_instance.state = Node.DEBUG_POST_TICK
            self.wait_for_continue()
            node_instance.state = old_state
            if node_instance.name in self._debug_settings_msg.breakpoint_names:
                self._debug_settings_msg.breakpoint_names.remove(node_instance.name)
                if self.publish_debug_settings:
                    self.publish_debug_settings(self._debug_settings_msg)

    def wait_for_continue(self):
        # If we have a publish callback, publish debug info
        if self.publish_debug_info:
            self.publish_debug_info(self.get_debug_info_msg())
        # Ensure that we're not picking up an extra continue request sent earlier
        self.continue_event.clear()
        self.continue_event.wait()
        self.continue_event.clear()

    def get_debug_info_msg(self):
        with self._lock:
            return deepcopy(self._debug_info_msg)
