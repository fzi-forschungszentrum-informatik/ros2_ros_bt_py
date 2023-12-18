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
from ros_bt_py_interfaces.srv import GetMessageFields
from ros_bt_py.package_manager import PackageManager

from typing import List


class TestPackageManager:
    @pytest.fixture
    def package_manager(self):
        return PackageManager(["/tmp"])

    @pytest.mark.parametrize(
        "msg_type, constant_values",
        [
            ("geometry_msgs/msg/Twist", []),
            (
                "ros_bt_py_interfaces/msg/Node",
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
        ],
    )
    def test_get_message_constant_fields_successful(
        self, package_manager: PackageManager, msg_type: str, constant_values: List[str]
    ):
        request = GetMessageFields.Request()
        request.service = False
        request.message_type = msg_type

        response = GetMessageFields.Response()
        response = package_manager.get_message_constant_fields_handler(
            request=request, response=response
        )
        assert response is not None
        assert response.success
        assert len(response.field_names) == len(constant_values)
        for constant in constant_values:
            assert constant in response.field_names

    def test_get_message_constant_fields_service(self, package_manager: PackageManager):
        request = GetMessageFields.Request()
        request.service = True

        response = GetMessageFields.Response()
        response = package_manager.get_message_constant_fields_handler(
            request=request, response=response
        )

        assert response is not None
        assert not response.success
        assert len(response.error_message) > 0

    @pytest.mark.parametrize(
        "msg_type", ["test_msgs/msg/Bla", "ros_bt_py_interfaces/msg/None"]
    )
    def test_get_message_constant_fields_invalid_msgs(
        self, package_manager: PackageManager, msg_type: str
    ):
        request = GetMessageFields.Request()
        request.message_type = msg_type
        request.service = False

        response = GetMessageFields.Response()
        response = package_manager.get_message_constant_fields_handler(
            request=request, response=response
        )

        assert response is not None
        assert not response.success
        assert len(response.error_message) > 0
