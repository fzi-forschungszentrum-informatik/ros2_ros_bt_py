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
from string import Formatter
import os

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import DictType, ListType, StringType
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


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
        inputs={"a": StringType(), "b": StringType()},
        outputs={"formatted_string": StringType()},
        max_children=0,
    )
)
class StringConcatenation(Leaf):
    """Concatenate strings a and b."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            do(
                Ok(a + b)
                for a in self.inputs.get_value_as("a", str)
                for b in self.inputs.get_value_as("b", str)
            )
            .and_then(lambda val: self.outputs.set_value("formatted_string", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"format_string": StringType(), "dict": DictType()},
        outputs={"formatted_string": StringType()},
        max_children=0,
    )
)
class FormatString(Leaf):
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        try:
            return (
                do(
                    Ok(myformatter.format(s, **d))
                    for s in self.inputs.get_value_as("format_string", str)
                    for d in self.inputs.get_value_as("dict", dict)
                )
                .and_then(lambda val: self.outputs.set_value("formatted_string", val))
                .map(lambda _: BTNodeState.SUCCEEDED)
            )
        except Exception:
            # TODO Shouldn't this return an error instead of just failed?
            return Ok(BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={
            "format_strings": ListType(element_type=StringType()),
            "dict": DictType(),
        },
        outputs={"formatted_strings": ListType(element_type=StringType())},
        max_children=0,
    )
)
class FormatStringList(Leaf):
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

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        try:
            return (
                do(
                    Ok([myformatter.format(s, **d) for s in s_l])
                    for s_l in self.inputs.get_value_as("format_strings", list[str])
                    for d in self.inputs.get_value_as("dict", dict)
                )
                .and_then(lambda val: self.outputs.set_value("formatted_strings", val))
                .map(lambda _: BTNodeState.SUCCEEDED)
            )
        except Exception:
            # TODO Shouldn't this return an error instead of just failed?
            return Ok(BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        inputs={"path": StringType()},
        outputs={"filename": StringType(), "extension": StringType()},
        max_children=0,
    )
)
class GetFileExtension(Leaf):
    """Return filename and extension of the provided path."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            do(Ok(os.path.splitext(p)) for p in self.inputs.get_value_as("path", str))
            .and_then(
                lambda fname_ext: self.outputs.set_multiple_values(
                    filename=fname_ext[0],
                    extension=fname_ext[1],
                )
            )
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
