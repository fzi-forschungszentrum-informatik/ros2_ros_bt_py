from typing import Dict
import pytest

import unittest.mock as mock
from std_srvs.srv import SetBool
from ros_bt_py.ros_nodes.message_converters import FieldsToMessage
from rclpy.time import Time
from ros_bt_py_interfaces.msg import Node as NodeMsg, UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException


@pytest.mark.parametrize(
    "message,fields",
    [
        (SetBool.Request, {"data": False}),
        (
            UtilityBounds,
            {
                "can_execute": False,
                "has_upper_bound_success": False,
                "upper_bound_success": 0.0,
                "has_lower_bound_success": False,
                "lower_bound_success": 0.0,
                "has_upper_bound_failure": False,
                "upper_bound_failure": 0.0,
                "has_lower_bound_failure": False,
                "lower_bound_failure": 0.0,
            },
        ),
    ],
)
def test_node_success(message: type, fields: Dict[str, type]):
    unavailable_service = FieldsToMessage(
        options={
            "output_type": message,
        },
    )
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    assert unavailable_service.inputs.__len__() == len(fields)
    print(f"{unavailable_service.inputs}")
    for field, value in fields.items():
        unavailable_service.inputs[field] = value
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEED
    assert unavailable_service.outputs.__len__() == 1
