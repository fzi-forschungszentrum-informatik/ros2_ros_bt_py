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
from typing import Optional, Dict
from rclpy.node import Node

from ros_bt_py_interfaces.msg import NodeState

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef

from ros_bt_py.custom_types import MathUnaryOperator, MathBinaryOperator
from ros_bt_py.custom_types import MathOperandType, MathUnaryOperandType


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"input_type": type, "output_type": type},
        inputs={"in": OptionRef("input_type")},
        outputs={"out": OptionRef("output_type")},
        max_children=0,
        tags=["math", "convert", "variable"],
    )
)
class Convert(Leaf):
    """
    Convert between certain types.

    Useful in many cases indeed.
    """

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(Convert, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )
        # check the possible conversions here

        if self.options["input_type"] is self.options["output_type"]:
            pass
        elif self.options["output_type"] is str:
            # that should almost always work
            pass
        elif self.options["input_type"] is float and self.options["output_type"] is int:
            self.logwarn("loss of precission in conversion from float to int")
        elif self.options["input_type"] is bool and self.options["output_type"] is int:
            self.loginfo("interpreting False as 0 and True as 1")
        elif self.options["input_type"] is int and self.options["output_type"] is bool:
            self.loginfo("interpreting 0 as False and != 0 as True")
        elif self.options["input_type"] in [int, float] and self.options[
            "output_type"
        ] in [int, float]:
            pass
        else:
            raise BehaviorTreeException(
                'Conversion between "%s" and "%s" not implemented'
                % (self.options["input_type"], self.options["output_type"])
            )

    def _do_setup(self):
        pass

    def _do_tick(self):
        if self.options["input_type"] is self.options["output_type"]:
            # passthrough
            self.outputs["out"] = self.inputs["in"]
        elif self.options["output_type"] is str:
            # that should almost always work
            self.outputs["out"] = str(self.inputs["in"])
        elif self.options["input_type"] is int and self.options["output_type"] is bool:
            if self.inputs["in"] == 0:
                self.outputs["out"] = False
            else:
                self.outputs["out"] = True
        elif self.options["input_type"] is bool and self.options["output_type"] is int:
            if self.inputs["in"]:
                self.outputs["out"] = 1
            else:
                self.outputs["out"] = 0
        elif self.options["input_type"] in [int, float] and self.options[
            "output_type"
        ] in [int, float]:
            if self.options["input_type"] is int:
                if self.options["output_type"] is float:
                    self.outputs["out"] = float(self.inputs["in"])
            elif self.options["input_type"] is float:
                if self.options["output_type"] is int:
                    self.outputs["out"] = int(self.inputs["in"])
        return NodeState.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"operand_type": MathOperandType, "operator": MathBinaryOperator},
        inputs={},
        outputs={},
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

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(Operation, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )
        self.operators = {}
        self.operators["add"] = operator.add
        self.operators["+"] = operator.add
        self.operators["and"] = operator.__and__
        self.operators["&"] = operator.__and__
        self.operators["div"] = operator.truediv
        self.operators["/"] = operator.truediv
        self.operators["floordiv"] = operator.floordiv
        self.operators["//"] = operator.floordiv
        self.operators["lshift"] = operator.lshift
        self.operators["<<"] = operator.lshift
        self.operators["mod"] = operator.mod
        self.operators["%"] = operator.mod
        self.operators["mul"] = operator.mul
        self.operators["*"] = operator.mul
        self.operators["or"] = operator.__or__
        self.operators["|"] = operator.__or__
        self.operators["pow"] = operator.pow
        self.operators["**"] = operator.pow
        self.operators["rshift"] = operator.rshift
        self.operators[">>"] = operator.rshift
        self.operators["sub"] = operator.sub
        self.operators["-"] = operator.sub
        self.operators["truediv"] = operator.truediv
        self.operators["xor"] = operator.xor
        self.operators["^"] = operator.xor

        if self.options["operator"].operator not in self.operators:
            raise BehaviorTreeException(
                f"Operator {self.options['operator'].operator} is not recognized."
            )

        self.operand_type = None

        if self.options["operand_type"].operand_type == "int":
            self.operand_type = int
        elif self.options["operand_type"].operand_type == "float":
            self.operand_type = float
        elif self.options["operand_type"].operand_type == "bool":
            self.operand_type = bool

        node_inputs = {}
        node_inputs["a"] = self.operand_type
        node_inputs["b"] = self.operand_type

        node_outputs = {}
        node_outputs["result"] = self.operand_type

        self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        pass

    def _do_tick(self):
        self.outputs["result"] = self.operators[self.options["operator"].operator](
            self.inputs["a"], self.inputs["b"]
        )
        return NodeState.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"operand_type": MathUnaryOperandType, "operator": MathUnaryOperator},
        inputs={},
        outputs={},
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

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[Node] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        super(UnaryOperation, self).__init__(
            options=options,
            debug_manager=debug_manager,
            subtree_manager=subtree_manager,
            name=name,
            ros_node=ros_node,
            succeed_always=succeed_always,
            simulate_tick=simulate_tick,
        )
        self.operators = {}
        self.operators["not"] = operator.not_
        self.operators["inv"] = operator.inv
        self.operators["~"] = operator.inv
        self.operators["neg"] = operator.neg
        self.operators["-"] = operator.neg
        self.operators["pos"] = operator.pos
        self.operators["+"] = operator.pos
        self.operators["exp"] = math.exp
        self.operators["expm1"] = math.expm1
        self.operators["log"] = math.log
        self.operators["log1p"] = math.log1p
        self.operators["log10"] = math.log10
        self.operators["ceil"] = math.ceil
        self.operators["fabs"] = math.fabs
        self.operators["factorial"] = math.factorial
        self.operators["floor"] = math.floor
        self.operators["sqrt"] = math.sqrt
        self.operators["acos"] = math.acos
        self.operators["asin"] = math.asin
        self.operators["atan"] = math.atan
        self.operators["acosh"] = math.acosh
        self.operators["asinh"] = math.asinh
        self.operators["atanh"] = math.atanh
        self.operators["cos"] = math.cos
        self.operators["sin"] = math.sin
        self.operators["tan"] = math.tan
        self.operators["cosh"] = math.cosh
        self.operators["sinh"] = math.sinh
        self.operators["tanh"] = math.tanh
        self.operators["degrees"] = math.degrees
        self.operators["radians"] = math.radians
        self.operators["erf"] = math.erf
        self.operators["erfc"] = math.erfc
        self.operators["gamma"] = math.gamma
        self.operators["lgamma"] = math.lgamma

        if self.options["operator"].operator not in self.operators:
            raise BehaviorTreeException(
                f"Operator {self.options['operator'].operator} is not recognized."
            )

        self.operand_type = None

        if self.options["operand_type"].operand_type == "int":
            self.operand_type = int
        elif self.options["operand_type"].operand_type == "float":
            self.operand_type = float

        node_inputs = {}
        node_inputs["in"] = self.operand_type

        node_outputs = {}
        node_outputs["result"] = self.operand_type

        self.node_config.extend(
            NodeConfig(
                options={}, inputs=node_inputs, outputs=node_outputs, max_children=0
            )
        )

        self._register_node_data(source_map=node_inputs, target_map=self.inputs)
        self._register_node_data(source_map=node_outputs, target_map=self.outputs)

    def _do_setup(self):
        pass

    def _do_tick(self):
        self.outputs["result"] = self.operators[self.options["operator"].operator](
            self.inputs["in"]
        )
        return NodeState.SUCCEEDED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeState.IDLE

    def _do_untick(self):
        return NodeState.IDLE
