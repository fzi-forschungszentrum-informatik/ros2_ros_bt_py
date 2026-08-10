# Copyright (c) 2026 FZI Forschungszentrum Informatik
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
#    * Neither the name of the copyright holder nor the names of its
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
from unittest.mock import MagicMock

import ros_bt_py.package_manager
from ros_bt_py.data_types import StringType, RosMessageType
from ros_bt_py.package_manager import PackageManager, to_message_type

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ros_bt_py_interfaces.msg import MessageTypes, NodeIO, Packages


class TestPackageManager:
    @pytest.fixture
    def package_manager(self):
        return PackageManager(["/tmp"])

    def test_data_type_to_message_type(self):
        message_type_msg = to_message_type(Header)
        assert message_type_msg is not None
        assert message_type_msg.name == "std_msgs/msg/Header"
        assert message_type_msg.type == RosMessageType(Header).serialize_type()

        fields_dict = {
            "stamp": RosMessageType(Time).serialize_type(),
            "frame_id": StringType().serialize_type(),
        }
        field: NodeIO
        for field in message_type_msg.fields:
            assert field.key in fields_dict.keys()
            assert field.type == fields_dict[field.key]

    def test_data_type_to_message_type_skips_uninstantiable_message(self, monkeypatch):
        def raise_on_init(message):
            raise TypeError("broken message constructor")

        monkeypatch.setattr(ros_bt_py.package_manager, "RosMessageType", raise_on_init)

        assert to_message_type(Header) is None

    def test_republish_lists_for_late_subscribers(self):
        message_publisher = MagicMock()
        package_publisher = MagicMock()
        message_publisher.get_subscription_count.return_value = 1
        package_publisher.get_subscription_count.return_value = 1
        manager = PackageManager(
            ["/tmp"],
            publish_message_list_callback=message_publisher,
            publish_packages_list_callback=package_publisher,
        )
        manager.message_types = MessageTypes()
        manager.packages = Packages()

        manager.republish_lists()

        message_publisher.publish.assert_called_once_with(manager.message_types)
        package_publisher.publish.assert_called_once_with(manager.packages)
