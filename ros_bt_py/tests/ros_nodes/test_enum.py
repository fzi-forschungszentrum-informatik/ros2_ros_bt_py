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
from ros_bt_py.ros_helpers import EnumValue
from ros_bt_py.ros_nodes.enum import Enum
from std_msgs.msg import String, Int64
from sensor_msgs.msg import BatteryState


@pytest.mark.parametrize(
    "message,enum_key,enum_value",
    [
        (NodeMsg, "IDLE", "IDLE"),
        (NodeMsg, "SHUTDOWN", "SHUTDOWN"),
        (BatteryState, "POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE", 7),
    ],
)
def test_node_success(message, enum_key, enum_value):
    enum_value_1 = EnumValue(enum_value=enum_key)
    enum_node = Enum(
        options={"ros_message_type": message, "constant_name": enum_value_1}
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert enum_node.outputs["out"] is not None
    assert enum_node.outputs["out"] == enum_value

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize("message", [(String), (Int64)])
def test_node_exception(message):
    enum_value = EnumValue(enum_value="")
    with pytest.raises(BehaviorTreeException):
        Enum(options={"ros_message_type": message, "constant_name": enum_value})


@pytest.mark.parametrize("message,enum_key", [(NodeMsg, "Tequila")])
def test_node_invalid_field(message, enum_key):
    enum_value = EnumValue(enum_value=enum_key)
    with pytest.raises(BehaviorTreeException):
        Enum(options={"ros_message_type": message, "constant_name": enum_value})


@pytest.mark.parametrize(
    "message,enum_key,enum_value",
    [
        (NodeMsg, "IDLE", "IDLE"),
        (NodeMsg, "SHUTDOWN", "SHUTDOWN"),
        (BatteryState, "POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE", 7),
    ],
)
def test_node_reset(message, enum_key, enum_value):
    enum_value_1 = EnumValue(enum_value=enum_key)
    enum_node = Enum(
        options={"ros_message_type": message, "constant_name": enum_value_1}
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert enum_node.outputs["out"] is not None
    assert enum_node.outputs["out"] == enum_value

    enum_node.reset()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert enum_node.outputs["out"] is not None
    assert enum_node.outputs["out"] == enum_value

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize(
    "message,enum_key,enum_value",
    [
        (NodeMsg, "IDLE", "IDLE"),
        (NodeMsg, "SHUTDOWN", "SHUTDOWN"),
        (BatteryState, "POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE", 7),
    ],
)
def test_node_untick(message, enum_key, enum_value):
    enum_value_1 = EnumValue(enum_value=enum_key)
    enum_node = Enum(
        options={"ros_message_type": message, "constant_name": enum_value_1}
    )
    enum_node.setup()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert enum_node.outputs["out"] is not None
    assert enum_node.outputs["out"] == enum_value

    enum_node.untick()
    assert enum_node.state == NodeMsg.IDLE

    enum_node.tick()
    assert enum_node.state == NodeMsg.SUCCEED
    assert enum_node.outputs["out"] is not None
    assert enum_node.outputs["out"] == enum_value

    enum_node.shutdown()
    assert enum_node.state == NodeMsg.SHUTDOWN
