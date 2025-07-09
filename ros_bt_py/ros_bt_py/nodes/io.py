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

from result import Result, Ok, Err
from ros_bt_py.node import IO, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.helpers import BTNodeState
from ros_bt_py.exceptions import BehaviorTreeException

# FIXME: input/output are exact copies of each other, move this code to a common implementation


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"io_type": type, "default": OptionRef("io_type")},
        inputs={"in": OptionRef("io_type")},
        outputs={"out": OptionRef("io_type")},
        max_children=0,
    )
)
class IOInputOption(IO):
    """
    Explicitly marks the input of a subtree.

    If no input is connected to `in`, the value provided via the `default` option is used.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _handle_inputs(self):
        """
        Overwrite the Nodes _handle_input() function, ignoring an unset 'in' input.

        Execute the callbacks registered by :meth:`_wire_input`
        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == "in" and self.inputs[input_name] is None:
                self.logdebug('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logdebug("Running tick() with stale data!")
        self.inputs.handle_subscriptions()
        return Ok(None)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.inputs["in"] is not None:
            self.outputs["out"] = self.inputs["in"]
        else:
            self.outputs["out"] = self.options["default"]
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["out"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"io_type": type},
        inputs={"in": OptionRef("io_type"), "default": OptionRef("io_type")},
        outputs={"out": OptionRef("io_type")},
        max_children=0,
    )
)
class IOInput(IO):
    """
    Explicitly marks the input of a subtree.

    If no input is connected to `in`, the value provided via the `default` input is used.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _handle_inputs(self):
        """
        Overwrite the Nodes _handle_input() function, ignoring an unset 'in' input.

        Execute the callbacks registered by :meth:`_wire_input`.
        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == "in" and self.inputs[input_name] is None:
                self.logdebug('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logdebug("Running tick() with stale data!")
                if self.inputs[input_name] is None:
                    return Err(ValueError(
                        f"Trying to tick a node with an unset input ({input_name})!"
                    ))
        self.inputs.handle_subscriptions()
        return Ok(None)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.inputs["in"] is not None:
            self.outputs["out"] = self.inputs["in"]
        else:
            self.outputs["out"] = self.inputs["default"]
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["out"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"io_type": type, "default": OptionRef("io_type")},
        inputs={"in": OptionRef("io_type")},
        outputs={"out": OptionRef("io_type")},
        max_children=0,
    )
)
class IOOutputOption(IO):
    """
    Explicitly marks the output of a subtree.

    If no input is connected to `in`, the value provided via the `default` option is used.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _handle_inputs(self):
        """
        Overwrite the Nodes _handle_input() function, ignoring an unset 'in' input.

        Execute the callbacks registered by :meth:`_wire_input`.
        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == "in" and self.inputs[input_name] is None:
                self.logdebug('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logdebug("Running tick() with stale data!")
        self.inputs.handle_subscriptions()
        return Ok(None)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.inputs["in"] is not None:
            self.outputs["out"] = self.inputs["in"]
        else:
            self.outputs["out"] = self.options["default"]
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["out"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"io_type": type},
        inputs={"in": OptionRef("io_type"), "default": OptionRef("io_type")},
        outputs={"out": OptionRef("io_type")},
        max_children=0,
    )
)
class IOOutput(IO):
    """
    Explicitly marks the output of a subtree.

    If no input is connected to `in`, the value provided via the `default` input is used.
    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _handle_inputs(self):
        """
        Overwrite the Nodes _handle_input() function, ignoring an unset 'in' input.

        Execute the callbacks registered by :meth:`_wire_input`.
        But only if an input has been updated since the last tick.
        """
        for input_name in self.inputs:
            if input_name == "in" and self.inputs[input_name] is None:
                self.logdebug('ignoring unset "in" input and using default value')
            else:
                if not self.inputs.is_updated(input_name):
                    self.logdebug("Running tick() with stale data!")
                if self.inputs[input_name] is None:
                    return Err(ValueError(
                        f"Trying to tick a node with an unset input ({input_name})!"
                    ))
        self.inputs.handle_subscriptions()
        return Ok(None)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if self.inputs["in"] is not None:
            self.outputs["out"] = self.inputs["in"]
        else:
            self.outputs["out"] = self.inputs["default"]
        return Ok(BTNodeState.SUCCEEDED)

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.IDLE)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["out"] = None
        self.outputs.reset_updated()
        return Ok(BTNodeState.IDLE)
