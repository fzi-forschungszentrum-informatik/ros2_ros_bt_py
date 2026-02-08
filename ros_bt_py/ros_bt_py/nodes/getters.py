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
"""BT nodes to get values from containers and other nodes."""

from result import Result, Ok, Err

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.helpers import BTNodeState, rgetattr
from ros_bt_py.exceptions import BehaviorTreeException


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"list_type": type, "index": int, "succeed_on_stale_data": bool},
        inputs={"list": list},
        outputs={"item": OptionRef("list_type")},
        max_children=1,
    )
)
class GetConstListItem(Decorator):
    """
    Extracts the item at the given `index` from `list`.

    The option parameter `succeed_on_stale_data` determines whether
    the node returns SUCCEEDED or RUNNING if `list` hasn't been
    updated since the last tick.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["list"] = []
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("list"):
            try:
                self.outputs["item"] = self.inputs["list"][self.options["index"]]
                return Ok(BTNodeState.SUCCEEDED)
            except IndexError:
                self.logerr(
                    "List index %d out of bound for list %s"
                    % (self.options["index"], self.inputs["list"])
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                # We don't need to check whether we have gotten any
                # data at all, because if we hadn't the tick method
                # would raise an error
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["item"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"list_type": type, "succeed_on_stale_data": bool},
        inputs={"list": list, "index": int},
        outputs={"item": OptionRef("list_type")},
        max_children=1,
    )
)
class GetListItem(Decorator):
    """
    Extracts the item at the given `index` from `list`.

    The option parameter `succeed_on_stale_data` determines whether
    the node returns SUCCEEDED or RUNNING if `list` hasn't been
    updated since the last tick.

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["list"] = []
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("list"):
            try:
                self.outputs["item"] = self.inputs["list"][self.inputs["index"]]
                return Ok(BTNodeState.SUCCEEDED)
            except IndexError:
                self.logerr(
                    "List index %d out of bound for list %s"
                    % (self.options["index"], self.inputs["list"])
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                # We don't need to check whether we have gotten any
                # data at all, because if we hadn't the tick method
                # would raise an error
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["item"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"value_type": type, "key": str, "succeed_on_stale_data": bool},
        inputs={"dict": dict},
        outputs={"value": OptionRef("value_type")},
        max_children=1,
    )
)
class GetDictItem(Decorator):
    """Get a item with a specific key from a dict input."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["dict"] = {}
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("dict"):
            try:
                self.outputs["value"] = self.inputs["dict"][self.options["key"]]
                return Ok(BTNodeState.SUCCEEDED)
            except KeyError:
                self.logwarn(
                    f"Key {self.options['key']} is not in dict {str(self.inputs['dict'])}"
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["value"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"keys": list, "succeed_on_stale_data": bool},
        inputs={"dict": dict},
        outputs={"values": list},
        max_children=1,
    )
)
class GetMultipleDictItems(Decorator):
    """Get multiple dict items with a specific list of keys."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["dict"] = {}
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("dict"):
            try:
                self.outputs["values"] = [
                    self.inputs["dict"][k] for k in self.options["keys"]
                ]
                return Ok(BTNodeState.SUCCEEDED)
            except KeyError:
                self.logwarn(
                    f"One of the key ({self.options['keys']}) is not in dict "
                    f"{str(self.inputs['dict'])}"
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["values"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"value_type": type, "dict": dict, "succeed_on_stale_data": bool},
        inputs={"key": str},
        outputs={"value": OptionRef("value_type")},
        max_children=1,
    )
)
class GetDictItemFromKey(Decorator):
    """Get a specific dict item with a key as data input."""

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["key"] = ""
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("key"):
            try:
                self.outputs["value"] = self.options["dict"][self.inputs["key"]]
                return Ok(BTNodeState.SUCCEEDED)
            except KeyError:
                self.logwarn(
                    f"Key {self.inputs['key']} is not in dict {str(self.options['dict'])}"
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["value"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"attr_type": type, "attr_name": str, "succeed_on_stale_data": bool},
        inputs={"object": object},
        outputs={"attr": OptionRef("attr_type")},
        max_children=1,
    )
)
class GetAttr(Decorator):
    """
    Get a specific attribute form a python object.

    This can also be done with nested attributes,
    e.g. the sec argument the timestamp of a
    std_msgs/msg/Header.msg can be extracted by using stamp.sec

    """

    def _do_setup(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].setup()
            if result.is_err():
                return result
        self.inputs["object"] = ""
        self.inputs.reset_updated()
        return Ok(BTNodeState.IDLE)

    def _do_tick(self) -> Result[BTNodeState, BehaviorTreeException]:
        # Tick child (if any) so it can produce its output before we process it
        if len(self.children) == 1:
            result = self.children[0].tick()
            if result.is_err():
                return result

            if result.ok() == BTNodeState.FAILED:
                return Ok(BTNodeState.FAILED)
            elif result.ok() == BTNodeState.RUNNING:
                return Ok(BTNodeState.RUNNING)

        if self.inputs.is_updated("object"):
            try:
                # TODO Maybe it would be nice to allow for calling 0-argument functions this way?
                self.outputs["attr"] = rgetattr(
                    self.inputs["object"], self.options["attr_name"]
                )
                return Ok(BTNodeState.SUCCEEDED)
            except AttributeError:
                self.logwarn(
                    f"Object {self.inputs['object']} does not have attribute "
                    f"{self.options['attr_name']}"
                )
                return Ok(BTNodeState.FAILED)
        else:
            if self.options["succeed_on_stale_data"]:
                return Ok(BTNodeState.SUCCEEDED)
            else:
                self.loginfo("No new data since last tick!")
                return Ok(BTNodeState.RUNNING)

    def _do_shutdown(self) -> Result[BTNodeState, BehaviorTreeException]:
        return Ok(BTNodeState.SHUTDOWN)

    def _do_reset(self) -> Result[BTNodeState, BehaviorTreeException]:
        self.outputs["attr"] = None
        self.outputs.reset_updated()
        return self._do_setup()

    def _do_untick(self) -> Result[BTNodeState, BehaviorTreeException]:
        if len(self.children) == 1:
            result = self.children[0].untick()
            if result.is_err():
                return result
        return Ok(BTNodeState.IDLE)
