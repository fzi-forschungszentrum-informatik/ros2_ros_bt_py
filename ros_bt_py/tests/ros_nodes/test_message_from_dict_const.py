import pytest
from ros_bt_py.ros_nodes.messages_from_dict import MessageFromConstDict
from std_msgs.msg import Int64, String
from std_srvs.srv import SetBool
from ros_bt_py_interfaces.srv import ChangeTreeName
from ros_bt_py_interfaces.msg import Node as NodeMsg


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (Int64, {"data": 667}),
        (String, {"data": "this is a test string"}),
        (SetBool.Request, {"data": True}),
    ],
)
def test_node_success(message, message_dict):
    node = MessageFromConstDict(options={"message_type": message, "dict": message_dict})
    node.setup()
    assert node.state == NodeMsg.IDLE
    node.tick()
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (Int64, {"data": 667}),
        (String, {"data": "this is a test string"}),
        (SetBool.Request, {"data": True}),
    ],
)
def test_node_untick(message, message_dict):
    node = MessageFromConstDict(options={"message_type": message, "dict": message_dict})
    node.setup()
    assert node.state == NodeMsg.IDLE

    node.tick()
    assert node.state == NodeMsg.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.untick()
    assert node.state == NodeMsg.IDLE

    node.tick()
    assert node.state == NodeMsg.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict",
    [
        (Int64, {"data": 667}),
        (String, {"data": "this is a test string"}),
        (SetBool.Request, {"data": True}),
    ],
)
def test_node_reset(message, message_dict):
    node = MessageFromConstDict(options={"message_type": message, "dict": message_dict})
    node.setup()
    assert node.state == NodeMsg.IDLE

    node.tick()
    assert node.state == NodeMsg.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.reset()
    assert node.state == NodeMsg.IDLE

    node.tick()
    assert node.state == NodeMsg.SUCCEED
    out_message = node.outputs["message"]
    assert out_message is not None
    for attr, attr_value in message_dict.items():
        assert getattr(out_message, attr) == attr_value

    node.shutdown()
    assert node.state == NodeMsg.SHUTDOWN


@pytest.mark.parametrize(
    "message, message_dict", [(ChangeTreeName.Request, {"tequila": False})]
)
def test_node_failure(message, message_dict):
    node = MessageFromConstDict(options={"message_type": message, "dict": message_dict})
    node.setup()
    assert node.state == NodeMsg.IDLE
    node.tick()
    assert node.state == NodeMsg.FAILURE

    out_message = node.outputs["message"]
    assert out_message is None
