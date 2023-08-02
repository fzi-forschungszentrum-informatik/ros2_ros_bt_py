import pytest

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py.ros_helpers import EnumValue
from ros_bt_py.ros_nodes.enum import EnumFields
from std_msgs.msg import String, Int64
from sensor_msgs.msg import BatteryState


@pytest.mark.parametrize("message,constants", [(NodeMsg, 17), (BatteryState, 21)])
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


@pytest.mark.parametrize("message,constants", [(NodeMsg, 17), (BatteryState, 21)])
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


@pytest.mark.parametrize("message,constants", [(NodeMsg, 17), (BatteryState, 21)])
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
