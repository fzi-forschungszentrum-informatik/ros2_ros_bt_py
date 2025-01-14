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
from typing import Dict
import pytest

from std_srvs.srv import SetBool
from ros_bt_py.custom_types import RosTopicType
from ros_bt_py.ros_nodes.message_converters import MessageToFields
from ros_bt_py_interfaces.msg import Node as NodeMsg, UtilityBounds


@pytest.mark.parametrize(
    "message,fields",
    [
        (RosTopicType("std_msgs/msg/Bool"), {"data": False}),
        (
            RosTopicType("ros_bt_py_interfaces/msg/UtilityBounds"),
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
    unavailable_service = MessageToFields(
        options={
            "input_type": message,
        },
    )
    unavailable_service.setup()
    assert unavailable_service.state == NodeMsg.IDLE
    unavailable_service.inputs["in"] = message.get_type_obj()()
    unavailable_service.tick()
    assert unavailable_service.state == NodeMsg.SUCCEED
    assert unavailable_service.outputs.__len__() == len(fields)
    print(f"{unavailable_service.outputs}")
    for field, value in fields.items():
        assert unavailable_service.outputs[field] == value
