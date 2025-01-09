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

from ros_bt_py.node import Leaf, define_bt_node, BTNodeState
from ros_bt_py.node_config import NodeConfig

from string import Formatter
import os
from result import Result, Ok, Err, is_err


class ExtendedFormatter(Formatter):
    """
    An extended format string formatter.

    Formatter with extended conversion symbol
    """

    def convert_field(self, value: str, conversion: str | None) -> str:
        """
        Extend conversion symbol.

        Following additional symbol has been added
        * l: convert to string and low case
        * u: convert to string and up case

        default are:
        * s: convert with str()
        * r: convert with repr()
        * a: convert with ascii()
        """
        if conversion == "u":
            return str(value).upper()
        elif conversion == "l":
            return str(value).lower()
        elif conversion == "c":
            return str(value).capitalize()

        # Do the default conversion or raise error if no matching conversion found
        super(ExtendedFormatter, self).convert_field(value, conversion)

        # return for None case
        return value


myformatter = ExtendedFormatter()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"a": str, "b": str},
        outputs={"formatted_string": str},
        max_children=0,
    )
)
class StringConcatenation(Leaf):
    """Concatenate strings a and b."""

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        self.outputs["formatted_string"] = self.inputs["a"] + self.inputs["b"]
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.outputs["formatted_string"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"format_string": str},
        inputs={"dict": dict},
        outputs={"formatted_string": str},
        max_children=0,
    )
)
class FormatOptionNode(Leaf):
    """
    Formats Dict to String based on option.

    Accepts a dictionary as input and outputs a formatted string
    based on the format string set in the options.

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_string: 'foo {first}'

    results in the following output:
    formatted_string: 'foo bar'
    """

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        try:
            self.outputs["formatted_string"] = myformatter.format(
                self.options["format_string"], **self.inputs["dict"]
            )
        except Exception:
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.outputs["formatted_string"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"dict": dict, "format_string": str},
        outputs={"formatted_string": str},
        max_children=0,
    )
)
class FormatInputNode(Leaf):
    """
    Formats Dict to List based on input setting.

    Accepts a dictionary and a format string as input and outputs a formatted string
    based on the format string

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_string: 'foo {first}'

    results in the following output:
    formatted_string: 'foo bar'
    """

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        try:
            self.outputs["formatted_string"] = myformatter.format(
                self.inputs["format_string"], **self.inputs["dict"]
            )
        except Exception:
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.outputs["formatted_string"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"format_strings": list},
        inputs={"dict": dict},
        outputs={"formatted_strings": list},
        max_children=0,
    )
)
class FormatOptionListNode(Leaf):
    """
    Formats Dict to List based on option setting.

    Accepts a dictionary as input and outputs a formatted strings in the list
    based on the format string set in the options.

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_strings: ['foo {first}', 'bar {first}']

    results in the following output:
    formatted_strings: ['foo bar', 'bar bar']
    """

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        try:
            self.outputs["formatted_strings"] = [
                myformatter.format(phrase, **self.inputs["dict"])
                for phrase in self.options["format_strings"]
            ]
        except Exception:
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.outputs["formatted_strings"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"dict": dict, "format_strings": list},
        outputs={"formatted_strings": list},
        max_children=0,
    )
)
class FormatInputListNode(Leaf):
    """
    Formats Input Dict into List based on format string.

    Accepts a dictionary and a list of format strings as input and
    outputs a list of formatted strings based on the format string

    Example dict and format_string:
    dict: {'first': 'bar', 'second': 'not_printed'}
    format_strings: ['foo {first}', 'bar {first}']

    results in the following output:
    formatted_strings: ['foo bar', 'bar bar']
    """

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        try:
            self.outputs["formatted_strings"] = [
                myformatter.format(phrase, **self.inputs["dict"])
                for phrase in self.inputs["format_strings"]
            ]
        except Exception:
            return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        self.outputs["formatted_strings"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"path": str},
        outputs={"filename": str, "extension": str},
        max_children=0,
    )
)
class GetFileExtension(Leaf):
    """Return filename and extension of the provided path."""

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.inputs.is_updated("path"):
            filename, extension = os.path.splitext(self.inputs["path"])
            self.outputs["extension"] = extension
            self.outputs["filename"] = filename
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self):
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self):
        return Ok(BTNodeState.IDLE)
