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
import operator
import math
from typing import Callable, Any

from typeguard import typechecked

from ros_bt_py.vendor.result import Result, Ok, Err, do

from ros_bt_py.data_types import (
    GenericType,
    ReferenceType,
    StringType,
)
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig


@typechecked
def get_conversion(
    in_type: type, out_type: type
) -> Result[Callable[[Any], Any], BehaviorTreeException]:
    if in_type is out_type:
        return Ok(lambda x: x)
    elif out_type is str:
        # that should almost always work
        return Ok(str)
    elif in_type is int and out_type is bool:
        return Ok(bool)
    elif in_type is bool and out_type is int:
        return Ok(int)
    elif in_type is int and out_type is float:
        return Ok(float)
    elif in_type is float and out_type is int:
        return Ok(int)
    else:
        return Err(
            BehaviorTreeException(
                'Conversion between "%s" and "%s" not implemented' % (in_type, out_type)
            )
        )


@define_bt_node(
    NodeConfig(
        inputs={
            "input_type": GenericType(),
            "output_type": GenericType(),
            "in": ReferenceType("input_type"),
        },
        outputs={"out": ReferenceType("output_type")},
        max_children=0,
        tags=["math", "convert", "variable"],
    )
)
class Convert(Leaf):
    """
    Convert between certain types.

    Useful in many cases indeed.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        match do(
            Ok((i, o, c))
            for i in self.inputs.get_value_as("input_type", type)
            for o in self.inputs.get_value_as("output_type", type)
            for c in self.inputs.get_value_as("clamp", bool)
        ):
            case Err(e):
                return Err(e)
            case Ok((i, o, c)):
                self.in_type = i
                self.out_type = o
                self.clamp = c

        match get_conversion(self.in_type, self.out_type):
            case Err(e):
                return Err(e)
            case Ok(c):
                self.conversion = c

        # Issue appropriate warnings
        if self.in_type is float and self.out_type is int:
            self.logwarn("loss of precission in conversion from float to int")
        elif self.in_type is bool and self.out_type is int:
            self.loginfo("interpreting False as 0 and True as 1")
        elif self.in_type is int and self.out_type is bool:
            self.loginfo("interpreting 0 as False and != 0 as True")

        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            self.inputs.get_value_as("in", self.in_type)
            .map(lambda val: self.conversion(val))
            .and_then(lambda val: self.outputs.set_value("out", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


BINARY_OPERATIONS = {
    "add": operator.add,
    "+": operator.add,
    "and": operator.__and__,
    "&": operator.__and__,
    "div": operator.truediv,
    "/": operator.truediv,
    "floordiv": operator.floordiv,
    "//": operator.floordiv,
    "lshift": operator.lshift,
    "<<": operator.lshift,
    "mod": operator.mod,
    "%": operator.mod,
    "mul": operator.mul,
    "*": operator.mul,
    "or": operator.__or__,
    "|": operator.__or__,
    "pow": operator.pow,
    "**": operator.pow,
    "rshift": operator.rshift,
    ">>": operator.rshift,
    "sub": operator.sub,
    "-": operator.sub,
    "truediv": operator.truediv,
    "xor": operator.xor,
    "^": operator.xor,
}


@define_bt_node(
    NodeConfig(
        inputs={
            "operand_type": GenericType(valid_types=[bool, int, float]),
            "operator": StringType(valid_values=list(BINARY_OPERATIONS.keys())),
            "a": ReferenceType(reference="operand_type"),
            "b": ReferenceType(reference="operand_type"),
        },
        outputs={"result": ReferenceType(reference="operand_type")},
        max_children=0,
        tags=[
            "math",
            "operator",
            "operation",
            "calculation",
            "result",
            "variable",
            "+-/*%",
            "add",
            "div",
            "sub",
            "mul",
            "mod",
            "and",
            "or",
            "xor",
            "shift",
            "pow",
        ],
    )
)
class Operation(Leaf):
    """Performs the desired binary operation on the inputs a and b."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            do(
                Ok(BINARY_OPERATIONS[s](a, b))
                for s in self.inputs.get_value_as("operator", str)
                for a in self.inputs.get_value("a")
                for b in self.inputs.get_value("b")
            )
            .and_then(lambda val: self.outputs.set_value("result", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)


UNARY_OPERATIONS = {
    "not": operator.not_,
    "inv": operator.inv,
    "~": operator.inv,
    "neg": operator.neg,
    "-": operator.neg,
    "pos": operator.pos,
    "+": operator.pos,
    "exp": math.exp,
    "expm1": math.expm1,
    "log": math.log,
    "log1p": math.log1p,
    "log10": math.log10,
    "ceil": math.ceil,
    "fabs": math.fabs,
    "factorial": math.factorial,
    "floor": math.floor,
    "sqrt": math.sqrt,
    "acos": math.acos,
    "asin": math.asin,
    "atan": math.atan,
    "acosh": math.acosh,
    "asinh": math.asinh,
    "atanh": math.atanh,
    "cos": math.cos,
    "sin": math.sin,
    "tan": math.tan,
    "cosh": math.cosh,
    "sinh": math.sinh,
    "tanh": math.tanh,
    "degrees": math.degrees,
    "radians": math.radians,
    "erf": math.erf,
    "erfc": math.erfc,
    "gamma": math.gamma,
    "lgamma": math.lgamma,
}


@define_bt_node(
    NodeConfig(
        inputs={
            "operand_type": GenericType(valid_types=[bool, int, float]),
            "operator": StringType(valid_values=list(UNARY_OPERATIONS.keys())),
            "in": ReferenceType(reference="operand_type"),
        },
        outputs={"result": ReferenceType(reference="operand_type")},
        max_children=0,
        tags=[
            "math",
            "operator",
            "operation",
            "calculation",
            "result",
            "variable",
            "not",
            "inv",
            "log",
            "ceil",
            "floor",
            "sqrt",
            "sin",
            "cos",
            "tan",
            "degrees",
            "radians",
            "error",
            "erf",
            "gamma",
        ],
    )
)
class UnaryOperation(Leaf):
    """Performs the desired unary operation on the inputs a and b."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return (
            do(
                Ok(UNARY_OPERATIONS[s](i))
                for s in self.inputs.get_value_as("operator", str)
                for i in self.inputs.get_value("in")
            )
            .and_then(lambda val: self.outputs.set_value("result", val))
            .map(lambda _: BTNodeState.SUCCEEDED)
        )

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)
