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
from ros_bt_py.helpers import (
    get_default_value,
)
import pytest
import unittest.mock as mock
from collections import OrderedDict
from ros_bt_py.ros_helpers import EnumValue, LoggerLevel


class TestHelpers:
    @pytest.mark.parametrize(
        "data_type, expected_output, ros",
        [
            (type, int, False),
            (int, 0, False),
            (str, "foo", False),
            (float, 1.2, False),
            (bool, False, False),
            (list, [], False),
            (dict, {}, False),
            (type, int, True),
            (list, [], True),
        ],
    )
    def test_get_default_value_basic_data_types(self, data_type, expected_output, ros):
        output = get_default_value(data_type, ros)
        assert output == expected_output

    @pytest.mark.parametrize(
        "data_type, ros",
        [
            (OrderedDict, False),
            (LoggerLevel, False),
            (EnumValue, False),
            (OrderedDict, True),
        ],
    )
    def test_get_default_value_defined_class_data_types(self, data_type, ros):
        output = get_default_value(data_type, ros)
        assert isinstance(output, data_type)

    @pytest.mark.parametrize(
        "data_type, expected_output, ros",
        [(mock.Mock(), mock.Mock, True), (mock.Mock(), {}, False)],
    )
    def test_get_default_value_other_class_data_types(
        self, data_type, expected_output, ros
    ):
        output = get_default_value(data_type, ros)
        if ros:
            assert isinstance(output, expected_output)
        else:
            assert output == expected_output
