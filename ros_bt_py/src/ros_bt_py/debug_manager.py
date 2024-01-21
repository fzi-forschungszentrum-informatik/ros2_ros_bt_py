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
from typing import Dict, Any


import rclpy.node

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue


from std_srvs.srv import SetBool


class DebugManager(object):
    def __init__(
        self,
        ros_node: rclpy.node.Node,
        node_diagnostics_publish_callback=None,
    ):
        self._lock = Lock()
        self._ros_node = ros_node

        self.publish_node_diagnostics = node_diagnostics_publish_callback

        self.diagnostics_state = {}
        self.diagnostics_state["SETUP"] = (
            "PRE_SETUP",
            "POST_SETUP",
        )
        self.diagnostics_state["UNTICK"] = (
            "PRE_UNTICK",
            "POST_UNTICK",
        )
        self.diagnostics_state["RESET"] = (
            "PRE_RESET",
            "POST_RESET",
        )
        self.diagnostics_state["SHUTDOWN"] = (
            "PRE_SHUTDOWN",
            "POST_SHUTDOWN",
        )

        self._collect_node_diagnostics = False

    def set_collect_node_diagnostics(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self._collect_node_diagnostics = request.data
        response.success = True
        return response

    def _dict_to_diagnostics_msg(self, diagnostics_dict: Dict[str, Any]):
        diagnostics_msg = DiagnosticStatus()
        diagnostics_msg.name = diagnostics_dict["name"]
        key_value_list = []
        for key, value in diagnostics_dict.items():
            if key != "name":
                key_value = KeyValue()
                key_value.key = key
                key_value.value = str(value)
                key_value_list.append(key_value)

        diagnostics_msg.values = key_value_list
        return diagnostics_msg

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
        diagnostics_msg = None
        if self._collect_node_diagnostics:
            diagnostics = {
                "name": node_instance.name,
                "stamp": self._ros_node.get_clock().now(),
                "module": type(node_instance).__module__,
                "node_class": type(node_instance).__name__,
                "path": [],
            }

            node = node_instance
            while node is not None:
                diagnostics["path"].append(node.name)
                node = node.parent
            # reverse the list
            diagnostics["path"] = diagnostics["path"][::-1]
            if self.publish_node_diagnostics:
                diagnostics_msg = self._dict_to_diagnostics_msg(diagnostics)
                self.publish_node_diagnostics(diagnostics_msg)

        # Contextmanager'ed code is executed here
        yield

        if self._collect_node_diagnostics:
            diagnostics["state"] = self.diagnostics_state[state][1]
            diagnostics["stamp"] = self._ros_node.get_clock().now()
            if self.publish_node_diagnostics:
                if self.publish_node_diagnostics:
                    diagnostics_msg = self._dict_to_diagnostics_msg(diagnostics)
                    self.publish_node_diagnostics(diagnostics_msg)

    @contextmanager
    def report_tick(self, node_instance):
        """
        Collect debug data during ticks from Node execution.

        It measures the time between the beginning and the end of the
        tick function (which includes the ticks of any children) and
        calculates a moving window average of execution times as well as
        a minimum and maximum value.

        :param instance: The node that's executing
        """
        diagnostics_msg = None
        if self._collect_node_diagnostics:
            diagnostics = {
                "name": node_instance.name,
                "stamp": self._ros_node.get_clock().now(),
                "module": type(node_instance).__module__,
                "node_class": type(node_instance).__name__,
                "state": "PRE_TICK",
                "path": [],
            }

            node = node_instance
            while node is not None:
                diagnostics["path"].append(node.name)
                node = node.parent
            # reverse the list
            diagnostics["path"] = diagnostics["path"][::-1]
            if self.publish_node_diagnostics:
                diagnostics_msg = self._dict_to_diagnostics_msg(diagnostics)
                self.publish_node_diagnostics(diagnostics_msg)

        # Contextmanager'ed code is executed here
        yield

        if self._collect_node_diagnostics:
            diagnostics["state"] = "POST_TICK"
            diagnostics["stamp"] = self._ros_node.get_clock().now()

            if self.publish_node_diagnostics:
                diagnostics_msg = self._dict_to_diagnostics_msg(diagnostics)
                self.publish_node_diagnostics(diagnostics_msg)
