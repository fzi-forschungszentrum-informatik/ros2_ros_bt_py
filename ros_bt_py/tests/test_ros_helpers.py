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
import unittest.mock as mock

import uuid

from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.ros_helpers import (
    ros_to_uuid,
    uuid_to_ros,
    wiring_has_id,
    get_interface_name,
    get_message_constant_fields,
)

from std_msgs.msg import Header
from example_interfaces.msg import String, Int32
from example_interfaces.srv import Trigger
from example_interfaces.action import Fibonacci
from ros_bt_py_interfaces.msg import NodeState, Wiring


class TestRosHelpers:
    @pytest.mark.parametrize(
        "ros_id, py_id",
        [
            (
                "12345678-1234-5678-abcd-123456789abc",
                uuid.UUID("12345678-1234-5678-abcd-123456789abc"),
            ),
            ("1234-12-34-56-123456", None),
        ],
    )
    def test_ros_to_uuid(self, ros_id, py_id):
        result = ros_to_uuid(ros_id)
        if py_id is None:
            assert result.is_err()
            return
        assert result.is_ok()
        assert result.unwrap() == py_id

    @pytest.mark.parametrize(
        "py_id, ros_id",
        [
            (
                uuid.UUID("12345678-1234-5678-abcd-123456789abc"),
                "12345678-1234-5678-abcd-123456789abc",
            ),
            (uuid.UUID(int=0), "00000000-0000-0000-0000-000000000000"),
        ],
    )
    def test_uuid_to_ros(self, py_id, ros_id):
        assert uuid_to_ros(py_id) == ros_id

    @pytest.fixture
    def wiring(self):
        wir = Wiring()
        wir.source.node_id = str(uuid.UUID(int=1))
        wir.source.data_key = "foo"
        wir.target.node_id = str(uuid.UUID(int=2))
        wir.target.data_key = "bar"
        return wir

    @pytest.mark.parametrize(
        "ros_id, match",
        [
            (uuid.UUID(int=1), True),
            (uuid.UUID(int=2), True),
            (uuid.UUID(int=3), False),
        ],
    )
    def test_wiring_has_id(self, wiring: Wiring, ros_id: uuid.UUID, match):
        assert wiring_has_id(wiring, ros_id) == match

    def test_wiring_has_id_broken(self, wiring: Wiring):
        wiring.source.node_id = "1234-12-34-56-123456"
        wiring.target.node_id = "1234-12-34-56-123456"
        assert wiring_has_id(wiring, uuid.UUID(int=0)) is False

    @pytest.mark.parametrize(
        "msg_type, msg_name",
        [
            (Header, "std_msgs/msg/Header"),
            (String, "example_interfaces/msg/String"),
            (Trigger, "example_interfaces/srv/Trigger"),
            (Trigger.Request, "example_interfaces/srv/Trigger_Request"),
            (Fibonacci, "example_interfaces/action/Fibonacci"),
            (Fibonacci.Goal, "example_interfaces/action/Fibonacci_Goal"),
            (Fibonacci.Feedback, "example_interfaces/action/Fibonacci_Feedback"),
        ],
    )
    def test_get_interface_name(self, msg_type, msg_name):
        assert get_interface_name(msg_type) == msg_name

    @pytest.mark.parametrize(
        "message_class, expected_members",
        [
            (
                NodeState,
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
                ],
            ),
            (Header, []),
            (String, []),
            (Int32, []),
        ],
    )
    def test_get_message_constant_fields_success(self, message_class, expected_members):
        result = get_message_constant_fields(message_class)
        assert result.is_ok()
        members = result.unwrap()
        assert len(members) == len(expected_members)
        assert sorted(members) == sorted(expected_members)

    def test_get_message_constant_fields_failure(self):
        message_class = mock.Mock()
        assert get_message_constant_fields(message_class).is_err()
