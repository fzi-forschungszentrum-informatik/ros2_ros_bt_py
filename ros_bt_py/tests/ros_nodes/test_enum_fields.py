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

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py.ros_nodes.enum import EnumFields
from std_msgs.msg import String, Int64


@pytest.mark.parametrize("message,constants", [(NodeMsg, 14)])
def test_node_success(message, constants):
    enum_node = EnumFields(
        options={
            "ros_message_type": message,
        }
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert len(enum_node.outputs) == constants

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize("message", [(String), (Int64)])
def test_node_exception(message):
    with pytest.raises(BehaviorTreeException):
        EnumFields(
            options={
                "ros_message_type": message,
            }
        )


@pytest.mark.parametrize("message,constants", [(NodeMsg, 14)])
def test_node_reset(message, constants):
    enum_node = EnumFields(
        options={
            "ros_message_type": message,
        }
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert len(enum_node.outputs) == constants

    enum_node.reset()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert len(enum_node.outputs) == constants

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize("message,constants", [(NodeMsg, 14)])
def test_node_untick(message, constants):
    enum_node = EnumFields(
        options={
            "ros_message_type": message,
        }
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert len(enum_node.outputs) == constants

    enum_node.untick()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert len(enum_node.outputs) == constants

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN
