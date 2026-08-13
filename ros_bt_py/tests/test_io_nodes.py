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

"""Tests for IOInput and IOOutput nodes."""

import uuid

from ros_bt_py.vendor.result import Ok, Err

from ros_bt_py.data_types import GenericType
from ros_bt_py.nodes.io import IOInput, IOOutput


class TestIOInput:
    """Tests for IOInput node."""

    def test_io_input_with_value(self):
        """Test IOInput with a value connected to 'in'."""
        node_id = uuid.UUID(int=1)

        io_input = IOInput(
            node_id=node_id,
            new_inputs={"io_type": GenericType(valid_types=[int])},
        )

        assert io_input.setup().is_ok()
        assert io_input.node_config.inputs["in"].set_value(42).is_ok()
        assert io_input.tick().is_ok()

        match io_input.node_config.outputs["out"].get_value():
            case Err(e):
                assert False, e
            case Ok(v):
                value = v
        assert value == 42

    def test_io_input_has_no_default_input(self):
        """Test that IOInput does not have a default input."""
        node_id = uuid.UUID(int=1)

        io_input = IOInput(
            node_id=node_id,
            new_inputs={"io_type": GenericType(valid_types=[int])},
        )

        assert "default" not in io_input.node_config.inputs
        assert "io_type" in io_input.node_config.inputs
        assert "in" in io_input.node_config.inputs
        assert "out" in io_input.node_config.outputs


class TestIOOutput:
    """Tests for IOOutput node."""

    def test_io_output_with_value(self):
        """Test IOOutput with a value connected to 'in'."""
        node_id = uuid.UUID(int=1)

        io_output = IOOutput(
            node_id=node_id,
            new_inputs={"io_type": GenericType(valid_types=[int])},
        )

        assert io_output.setup().is_ok()
        assert io_output.node_config.inputs["in"].set_value(42).is_ok()
        assert io_output.tick().is_ok()

        match io_output.node_config.outputs["out"].get_value():
            case Err(e):
                assert False, e
            case Ok(v):
                value = v
        assert value == 42

    def test_io_output_no_default_input(self):
        """Test that IOOutput does not have a 'default' input."""
        node_id = uuid.UUID(int=1)

        io_output = IOOutput(
            node_id=node_id,
            new_inputs={"io_type": GenericType(valid_types=[int])},
        )

        assert "default" not in io_output.node_config.inputs
        assert "io_type" in io_output.node_config.inputs
        assert "in" in io_output.node_config.inputs
        assert "out" in io_output.node_config.outputs
