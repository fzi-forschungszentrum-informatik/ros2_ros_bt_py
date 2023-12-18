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
