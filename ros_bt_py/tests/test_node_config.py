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

from ros_bt_py.data_types import IntType, FloatType, GenericType, ReferenceType
from ros_bt_py.exceptions import NodeConfigError
from ros_bt_py.node_config import NodeConfig


@pytest.fixture
def example_inputs():
    return {
        "tinput": GenericType(),
        "input1": IntType(),
        "input2": ReferenceType(reference="tinput"),
    }


@pytest.fixture
def example_outputs():
    return {
        "output1": FloatType(),
        "output2": ReferenceType(reference="tinput"),
    }


@pytest.fixture
def example_max_children():
    return 3


@pytest.fixture
def example_tags():
    return ["tag1", "tag2"]


class TestNodeConfig:
    def test_init(self, example_inputs, example_outputs, example_max_children):
        node_config = NodeConfig(
            inputs=example_inputs,
            outputs=example_outputs,
            max_children=example_max_children,
        )
        assert node_config.inputs == example_inputs
        assert node_config.outputs == example_outputs
        assert node_config.max_children == example_max_children
        assert node_config.tags == []

    def test_repr(
        self,
        example_inputs,
        example_outputs,
        example_max_children,
        example_tags,
    ):
        node_config = NodeConfig(
            inputs=example_inputs,
            outputs=example_outputs,
            max_children=example_max_children,
            tags=example_tags,
        )

        expected_repr = (
            f"NodeConfig(inputs={example_inputs}, outputs={example_outputs}, "
            f"max_children={example_max_children})"
        )
        assert repr(node_config) == expected_repr

    @staticmethod
    @pytest.mark.parametrize(
        "config1, config2, result",
        [
            (
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                True,
            ),
            (
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=4,
                ),
                False,
            ),
            (
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                NodeConfig(
                    inputs={"input2": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                False,
            ),
            (
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": FloatType()},
                    max_children=3,
                ),
                NodeConfig(
                    inputs={"input1": IntType()},
                    outputs={"output1": IntType()},
                    max_children=3,
                ),
                False,
            ),
        ],
    )
    def test_eq(config1, config2, result):
        assert (config1 == config2) == result

    def test_extend_success(self):
        base_config = NodeConfig(
            inputs={"input1": IntType(), "input2": FloatType()},
            outputs={"output1": FloatType(), "output2": IntType()},
            max_children=3,
        )
        extension_config = NodeConfig(
            inputs={"input3": IntType(), "input4": FloatType()},
            outputs={"output3": FloatType(), "output4": IntType()},
            max_children=3,
        )
        assert base_config.extend(extension_config).is_ok()
        assert base_config.inputs == {
            "input1": IntType(),
            "input2": FloatType(),
            "input3": IntType(),
            "input4": FloatType(),
        }
        assert base_config.outputs == {
            "output1": FloatType(allow_static=False),
            "output2": IntType(allow_static=False),
            "output3": FloatType(allow_static=False),
            "output4": IntType(allow_static=False),
        }
        assert base_config.max_children == 3

    def test_extend_diff_max_childs(self):
        base_config = NodeConfig(
            inputs={"input1": IntType(), "input2": FloatType()},
            outputs={"output1": FloatType(), "output2": IntType()},
            max_children=3,
        )
        extension_config = NodeConfig(
            inputs={"input3": IntType(), "input4": FloatType()},
            outputs={"output3": FloatType(), "output4": IntType()},
            max_children=4,
        )
        result = base_config.extend(extension_config)
        assert result.is_err()
        assert isinstance(result.unwrap_err(), NodeConfigError)

    def test_extend_duplicate_input(self):
        base_config = NodeConfig(
            inputs={"input1": IntType(), "input2": FloatType()},
            outputs={"output1": FloatType(), "output2": IntType()},
            max_children=3,
        )
        extension_config = NodeConfig(
            inputs={"input1": IntType(), "input4": FloatType()},
            outputs={"output3": FloatType(), "output4": IntType()},
            max_children=3,
        )
        result = base_config.extend(extension_config)
        assert result.is_err()
        assert isinstance(result.unwrap_err(), NodeConfigError)

    def test_extend_duplicate_output(self):
        base_config = NodeConfig(
            inputs={"input1": IntType(), "input2": FloatType()},
            outputs={"output1": FloatType(), "output2": IntType()},
            max_children=3,
        )
        extension_config = NodeConfig(
            inputs={"input3": IntType(), "input4": FloatType()},
            outputs={"output3": FloatType(), "output2": IntType()},
            max_children=3,
        )
        result = base_config.extend(extension_config)
        assert result.is_err()
        assert isinstance(result.unwrap_err(), NodeConfigError)
