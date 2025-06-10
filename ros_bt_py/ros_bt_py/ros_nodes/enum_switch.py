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

from ros_bt_py_interfaces.msg import NodeState
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.node import FlowControl, define_bt_node
from ros_bt_py.node_config import NodeConfig

from ros_bt_py.custom_types import TypeWrapper, RosTopicType, ROS_TYPE_FULL
from ros_bt_py.ros_helpers import EnumValue, get_message_constant_fields
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"ros_message_type": TypeWrapper(RosTopicType, info=ROS_TYPE_FULL)},
        inputs={"case": int},
        outputs={},
        max_children=None,
    )
)
class EnumSwitch(FlowControl):
    def __init__(
        self,
        options=None,
        debug_manager=None,
        subtree_manager=None,
        name=None,
        ros_node=None,
        succeed_always=False,
        simulate_tick=False,
    ):
        super(EnumSwitch, self).__init__(
            options,
            debug_manager,
            subtree_manager,
            name,
            ros_node,
            succeed_always,
            simulate_tick,
        )

        self._message_class = self.options["ros_message_type"].get_type_obj()

        self.possible_children = get_message_constant_fields(self._message_class)

        if not self.possible_children:
            raise BehaviorTreeException(
                "%s has no constant fields" % (self.options["ros_message_type"])
            )

        self.msg = self._message_class()

        self.pchild_dict = dict(
            zip(
                [getattr(self.msg, entry) for entry in self.possible_children],
                self.possible_children,
            )
        )

    def _do_setup(self):
        self.child_map = {child.name.split(".")[-1]: child for child in self.children}
        for child in self.children:
            if child.name.split(".")[-1] not in self.possible_children:
                self.logerr("Unwanted child detected please fix name")
            child.setup()

    def _do_tick(self):
        name = self.inputs["case"]

        if name in self.pchild_dict.keys():
            e_name = self.pchild_dict[name]
        else:
            self.logwarn("Input did not match possible children.")
            return NodeState.FAILED

        if e_name not in self.child_map:
            self.logwarn(
                "Ticking without matching child. Is this really what you want?"
            )
            return NodeState.FAILED

        # If we've previously succeeded or failed, untick all children
        if self.state in [NodeState.SUCCEEDED, NodeState.FAILED]:
            for child in self.children:
                child.reset()

        for child_name, child in self.child_map.items():
            if not child_name == e_name:
                child.untick()

        return self.child_map[e_name].tick()

    def _do_untick(self):
        for child in self.children:
            child.untick()
        return NodeState.IDLE

    def _do_reset(self):
        for child in self.children:
            child.reset()
        return NodeState.IDLE

    def _do_shutdown(self):
        for child in self.children:
            child.shutdown()
