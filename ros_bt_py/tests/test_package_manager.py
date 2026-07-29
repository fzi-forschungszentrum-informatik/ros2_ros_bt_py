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

from ros_bt_py.data_types import StringType, RosMessageType
from ros_bt_py.package_manager import PackageManager, to_message_type

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from ros_bt_py_interfaces.msg import NodeIO


class TestPackageManager:
    @pytest.fixture
    def package_manager(self):
        return PackageManager(["/tmp"])

    def test_data_type_to_message_type(self):
        message_type_msg = to_message_type(Header)
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
