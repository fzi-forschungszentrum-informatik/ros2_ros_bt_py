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
"""BT nodes to convert Dicts to ROS messages of a specific type."""
from ros_bt_py_interfaces.msg import Node as NodeMsg
from rosidl_runtime_py.set_message import set_message_fields

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"message_type": type},
        inputs={"dict": dict},
        outputs={"message": OptionRef("message_type")},
        max_children=0,
    )
)
class MessageFromDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.inputs.is_updated("dict") or self.state == NodeMsg.IDLE:
            message = self.options["message_type"]()
            try:
                set_message_fields(
                    message,
                    self.inputs["dict"],
                )
                self.outputs["message"] = message
            except (
                AttributeError,
                TypeError,
            ) as ex:
                self.logerr(
                    f"Error populating message of type {self.options['message_type'].__name__}: "
                    f"{str(ex)}"
                )
                return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


@define_bt_node(
    NodeConfig(
        version="0.9.0",
        options={"message_type": type, "dict": dict},
        inputs={},
        outputs={"message": OptionRef("message_type")},
        max_children=0,
    )
)
class MessageFromConstDict(Leaf):
    """Fill a ROS message with the values from `dict`."""

    def _do_setup(self):
        pass

    def _do_tick(self):
        message = self.options["message_type"]()
        try:
            set_message_fields(
                message,
                self.options["dict"],
            )
            self.outputs["message"] = message
        except (TypeError, AttributeError) as ex:
            self.logerr(
                f"Error populating message of type {self.options['message_type'].__name__}: "
                f"{str(ex)}"
            )
            return NodeMsg.FAILED
        return NodeMsg.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
