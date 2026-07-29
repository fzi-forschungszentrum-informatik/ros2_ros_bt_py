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

from ros_bt_py.helpers import int_limits_dict, float_limits_dict, rgetattr, rsetattr


class TestHelpers:

    @pytest.mark.parametrize(
        "key, min_v, max_v",
        [
            ("int8", -(2**7), 2**7 - 1),
            ("int16", -(2**15), 2**15 - 1),
            ("int32", -(2**31), 2**31 - 1),
            ("int64", -(2**63), 2**63 - 1),
            ("uint8", 0, 2**8 - 1),
            ("uint16", 0, 2**16 - 1),
            ("uint32", 0, 2**32 - 1),
            ("uint64", 0, 2**64 - 1),
        ],
    )
    def test_int_limits_dict(self, key, min_v, max_v):
        limits = int_limits_dict(key)
        assert limits["min_value"] == min_v
        assert limits["max_value"] == max_v

    @pytest.mark.parametrize(
        "key, min_v, max_v",
        [
            ("float", -3.4028235e38, 3.4028235e38),
            ("double", -1.7976931348623157e308, 1.7976931348623157e308),
        ],
    )
    def test_float_limits_dict(self, key, min_v, max_v):
        limits = float_limits_dict(key)
        assert limits["min_value"] == min_v
        assert limits["max_value"] == max_v

    @pytest.mark.parametrize(
        "obj, attr_name, attr_val",
        [
            (
                type("a", (), {"a": 1})(),
                "a",
                1,
            ),
            (
                type("a", (), {"a": type("b", (), {"b": 1})()})(),
                "a.b",
                1,
            ),
        ],
    )
    def test_rgetattr(self, obj, attr_name, attr_val):
        assert rgetattr(obj, attr_name) == attr_val

    @pytest.mark.parametrize(
        "obj, attr_name, attr_val",
        [
            (
                type("a", (), {"a": 1})(),
                "a",
                2,
            ),
            (
                type("a", (), {"a": type("b", (), {"b": 1})()})(),
                "a.b",
                2,
            ),
        ],
    )
    def test_rsetattr(self, obj, attr_name, attr_val):
        rsetattr(obj, attr_name, attr_val)
        assert rgetattr(obj, attr_name) == attr_val
