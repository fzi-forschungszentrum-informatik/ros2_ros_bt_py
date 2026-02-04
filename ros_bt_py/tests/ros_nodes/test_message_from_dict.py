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
import pytest

from ros_bt_py.custom_types import RosTopicType
from ros_bt_py.ros_nodes.messages_from_dict import MessageFromDict
from ros_bt_py_interfaces.msg import NodeState


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (RosTopicType("std_msgs/msg/Int64"), {"data": 667}),
        (RosTopicType("std_msgs/msg/String"), {"data": "this is a test string"}),
        (RosTopicType("std_msgs/msg/Bool"), {"data": True}),
    ],
)
class TestMessageFromDict:

    @pytest.fixture
    def target_node(self, logging_mock, message):
        node = MessageFromDict(
            options={"message_type": message},
            logging_manager=logging_mock,
        )
        return node

    def test_node_success(self, target_node, message_dict):
        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE
        target_node.inputs["dict"] = message_dict

        assert target_node.tick().is_ok()
        out_message = target_node.outputs["message"]
        assert out_message is not None
        for attr, attr_value in message_dict.items():
            assert getattr(out_message, attr) == attr_value

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_untick(self, target_node, message_dict):
        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE
        target_node.inputs["dict"] = message_dict

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.SUCCEED
        out_message = target_node.outputs["message"]
        assert out_message is not None
        for attr, attr_value in message_dict.items():
            assert getattr(out_message, attr) == attr_value

        assert target_node.untick().is_ok()
        assert target_node.state == NodeState.IDLE

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.SUCCEED
        out_message = target_node.outputs["message"]
        assert out_message is not None
        for attr, attr_value in message_dict.items():
            assert getattr(out_message, attr) == attr_value

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_reset(self, target_node, message_dict):
        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE
        target_node.inputs["dict"] = message_dict

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.SUCCEED
        out_message = target_node.outputs["message"]
        assert out_message is not None
        for attr, attr_value in message_dict.items():
            assert getattr(out_message, attr) == attr_value

        assert target_node.reset().is_ok()
        assert target_node.state == NodeState.IDLE

        assert target_node.tick().is_ok()
        assert target_node.state == NodeState.SUCCEED
        out_message = target_node.outputs["message"]
        assert out_message is not None
        for attr, attr_value in message_dict.items():
            assert getattr(out_message, attr) == attr_value

        assert target_node.shutdown().is_ok()
        assert target_node.state == NodeState.SHUTDOWN

    def test_node_failure(self, target_node, message_dict, error_log):
        assert target_node.setup().is_ok()
        assert target_node.state == NodeState.IDLE

        # Set message dict to junk value
        target_node.inputs["dict"] = {"tequila": False}
        with pytest.warns(error_log, match=".*Error populating message.*"):
            assert target_node.tick().is_ok()
        assert target_node.state == NodeState.FAILURE

        out_message = target_node.outputs["message"]
        assert out_message is None
