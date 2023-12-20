import pytest
from ros_bt_py.ros_helpers import get_message_constant_fields
from std_msgs.msg import Header, String, Int32
from ros_bt_py_interfaces.msg import Node, NodeDataLocation
from ros_bt_py.exceptions import BehaviorTreeException
import unittest.mock as mock


class TestRosHelpers:
    @pytest.mark.parametrize(
        "message_class, expected_members",
        [
            (
                Node,
                [
                    "UNINITIALIZED",
                    "IDLE",
                    "UNASSIGNED",
                    "ASSIGNED",
                    "RUNNING",
                    "SUCCEEDED",
                    "SUCCEED",
                    "SUCCESS",
                    "FAILED",
                    "FAIL",
                    "FAILURE",
                    "BROKEN",
                    "PAUSED",
                    "SHUTDOWN",
                    "DEBUG_PRE_TICK",
                    "DEBUG_TICK",
                    "DEBUG_POST_TICK",
                ],
            ),
            (NodeDataLocation, ["INPUT_DATA", "OUTPUT_DATA", "OPTION_DATA"]),
            (Header, []),
            (String, []),
            (Int32, []),
        ],
    )
    def test_get_message_constant_fields_no_exception(
        self, message_class, expected_members
    ):
        members = get_message_constant_fields(message_class)
        assert len(members) == len(expected_members)
        assert sorted(members) == sorted(expected_members)

    def test_get_message_constant_fields_exception(self):
        message_class = mock.Mock()
        with pytest.raises(BehaviorTreeException):
            get_message_constant_fields(message_class)
