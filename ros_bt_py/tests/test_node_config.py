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
from ros_bt_py.node_config import OptionRef, NodeConfig


@pytest.mark.parametrize("option_key", ["Test", 1, 1.0, True])
class TestOptionRefInputs:
    @staticmethod
    def test_init(option_key):
        option_ref = OptionRef(option_key)
        assert option_ref.option_key == option_key

    @staticmethod
    def test_repr(option_key):
        option_ref = OptionRef(option_key)
        expected_repr = f"OptionRef(option_key={option_key!r})"
        assert repr(option_ref) == expected_repr

    @staticmethod
    def test_name(option_key):
        option_ref = OptionRef(option_key)
        expected_name = f"OptionRef(option_key={option_key!r})"
        assert option_ref.__name__() == expected_name


class TestOptionRefEqs:
    @staticmethod
    @pytest.mark.parametrize(
        "same, other, result",
        [
            (OptionRef("Same"), OptionRef("Same"), True),
            (OptionRef("Same"), OptionRef("Other"), False),
            (OptionRef("Other"), OptionRef("Same"), False),
            (OptionRef("Other"), OptionRef("Other"), True),
        ],
    )
    def test_eq(same, other, result):
        assert (same == other) == result

    @staticmethod
    @pytest.mark.parametrize(
        "same, other, result",
        [
            (OptionRef("Same"), OptionRef("Same"), False),
            (OptionRef("Same"), OptionRef("Other"), True),
            (OptionRef("Other"), OptionRef("Same"), True),
            (OptionRef("Other"), OptionRef("Other"), False),
        ],
    )
    def test_ne(same, other, result):
        assert (same != other) == result


@pytest.fixture
def example_inputs():
    return {"input1": int, "input2": OptionRef(1)}


@pytest.fixture
def example_outputs():
    return {"output1": float, "output2": OptionRef("Test")}


@pytest.fixture
def example_options():
    return {"option1": str, "options2": bool}


@pytest.fixture
def example_optional_options():
    return {"optional1": str, "optional2": int}


@pytest.fixture
def example_max_children():
    return 3


@pytest.fixture
def example_version():
    return "1.0"


@pytest.fixture
def example_tags():
    return ["tag1", "tag2"]


class TestNodeConfig:
    def test_init_required(
        example_inputs, example_outputs, example_options, example_max_children
    ):
        node_config = NodeConfig(
            options=example_options,
            inputs=example_inputs,
            outputs=example_outputs,
            max_children=example_max_children,
        )
        assert node_config.inputs == example_inputs
        assert node_config.outputs == example_outputs
        assert node_config.options == example_options
        assert node_config.max_children == example_max_children
        assert node_config.optional_options == []
        assert node_config.tags == []
        assert node_config.version == ""

    def test_init_optional(
        example_inputs,
        example_outputs,
        example_options,
        example_optional_options,
        example_max_children,
        example_version,
        example_tags,
    ):

        node_config = NodeConfig(
            options=example_options,
            inputs=example_inputs,
            outputs=example_outputs,
            max_children=example_max_children,
            optional_options=example_optional_options,
            version=example_version,
            tags=example_tags,
        )

        assert node_config.inputs == example_inputs
        assert node_config.outputs == example_outputs
        assert node_config.options == example_options
        assert node_config.max_children == example_max_children
        assert node_config.optional_options == example_optional_options
        assert node_config.tags == example_tags
        assert node_config.version == example_version
