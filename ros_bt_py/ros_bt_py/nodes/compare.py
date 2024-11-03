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
from result import Result, Ok
from ros_bt_py.exceptions import BehaviorTreeException
from ros_bt_py.helpers import BTNodeState

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"compare_type": type},
        inputs={"a": OptionRef("compare_type"), "b": OptionRef("compare_type")},
        outputs={},
        max_children=0,
    )
)
class Compare(Leaf):
    """
    Compares `a` and `b`.

    Will succeed if `a == b` and fail otherwise
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.inputs["a"] == self.inputs["b"]:
            return Ok(BTNodeState.SUCCEEDED)

        # If we didn't received both values yet, or we did and they're
        # not equal, fail
        return Ok(BTNodeState.FAILED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"compare_type": type},
        inputs={"a": OptionRef("compare_type"), "b": OptionRef("compare_type")},
        outputs={},
        max_children=0,
    )
)
class CompareNewOnly(Leaf):
    """
    Compares `a` and `b`, but only if at least one of them has been updated this tick.

    Will succeed if `a == b` and fail otherwise
    """

    def _do_setup(self):
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if self.inputs.is_updated("a") or self.inputs.is_updated("b"):
            if self.inputs["a"] == self.inputs["b"]:
                return Ok(BTNodeState.SUCCEEDED)
            else:
                return Ok(BTNodeState.FAILED)
        return Ok(BTNodeState.RUNNING)

    def _do_untick(self):
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self):
        # Reset output to False, so we'll return False until we
        # receive a new input.
        self.inputs.reset_updated()

        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"compare_type": type, "expected": OptionRef("compare_type")},
        inputs={"in": OptionRef("compare_type")},
        outputs={},
        max_children=0,
    )
)
class CompareConstant(Leaf):
    """
    Compares `expected` and `in`.

    Will succeed if `expected == in` and fail otherwise
    """

    def _do_setup(self):
        self._received_in = False
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if not self._received_in:
            if self.inputs.is_updated("in"):
                self._received_in = True

        if self._received_in and self.options["expected"] == self.inputs["in"]:
            return Ok(BTNodeState.SUCCEEDED)
        else:
            return Ok(BTNodeState.FAILED)

    def _do_untick(self):
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self):
        self.inputs.reset_updated()
        self._received_in = False

        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={},
        inputs={"a": float, "b": float},
        outputs={},
        max_children=0,
    )
)
class ALessThanB(Leaf):
    """
    Compares `a` and `b`.

    Will succeed if `a < b` and fail otherwise
    """

    def _do_setup(self):
        self._received_a = False
        self._received_b = False
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if not self._received_a:
            if self.inputs.is_updated("a"):
                self._received_a = True
        if not self._received_b:
            if self.inputs.is_updated("b"):
                self._received_b = True
        if self._received_a and self._received_b:
            if self.inputs["a"] < self.inputs["b"]:
                return Ok(BTNodeState.SUCCEEDED)

        # If we didn't received both values yet, or we did and they're
        # not equal, fail
        return Ok(BTNodeState.FAILED)

    def _do_untick(self):
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self):
        self.inputs.reset_updated()
        self._received_a = False
        self._received_b = False
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


class LessThanConstantImp:
    def _do_setup(self):
        self._received_in = False
        return Ok(BTNodeState.IDLE)

    def _do_tick(self):
        if not self._received_in:
            if self.inputs.is_updated("a"):
                self._received_in = True

        if self._received_in and self.inputs["a"] < self.options["target"]:
            return Ok(BTNodeState.SUCCEEDED)
        else:
            return Ok(BTNodeState.FAILED)

    def _do_untick(self):
        # Nothing to do
        return Ok(BTNodeState.IDLE)

    def _do_reset(self):
        self.inputs.reset_updated()
        self._received_in = False

        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self):
        # Nothing to do
        return Ok(BTNodeState.SHUTDOWN)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"target": float},
        inputs={"a": float},
        outputs={},
        max_children=0,
    )
)
class LessThanConstant(LessThanConstantImp, Leaf):
    """
    Compares `a` and `target`.

    Will succeed if `a < target` and fail otherwise
    """

    pass


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"target": int},
        inputs={"a": int},
        outputs={},
        max_children=0,
    )
)
class LessThanIntConstant(LessThanConstantImp, Leaf):
    """
    Compares `a` and `target`.

    Will succeed if `a < target` and fail otherwise
    """

    pass
