# Copyright 2018-2023 FZI Forschungszentrum Informatik

"""BT nodes to get values from containers and other nodes."""

from ros_bt_py_interfaces.msg import Node as NodeMsg

from ros_bt_py.node import Decorator, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.helpers import rgetattr


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

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set list to an empty list. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["list"] = []
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("list"):
            try:
                self.outputs["item"] = self.inputs["list"][self.options["index"]]
                return NodeMsg.SUCCEEDED
            except IndexError:
                self.logerr(
                    "List index %d out of bound for list %s"
                    % (self.options["index"], self.inputs["list"])
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                # We don't need to check whether we have gotten any
                # data at all, because if we hadn't the tick method
                # would raise an error
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["item"] = None
        self.outputs.reset_updated()
        self._do_setup()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


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

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set list to an empty list. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["list"] = []
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("list") or self.inputs.is_updated("index"):
            try:
                self.outputs["item"] = self.inputs["list"][self.inputs["index"]]
                return NodeMsg.SUCCEEDED
            except IndexError:
                self.logerr(
                    "List index %d out of bound for list %s"
                    % (self.inputs["index"], self.inputs["list"])
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["item"] = None
        self.outputs.reset_updated()
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


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

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set dict to an empty dict. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["dict"] = {}
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("dict"):
            try:
                self.outputs["value"] = self.inputs["dict"][self.options["key"]]
                return NodeMsg.SUCCEEDED
            except KeyError:
                self.logerr(
                    f"Key {self.options['key']} is not in dict {str(self.inputs['dict'])}"
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["value"] = None
        self.outputs.reset_updated()
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


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

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set dict to an empty dict. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["dict"] = {}
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("dict"):
            try:
                self.outputs["values"] = [
                    self.inputs["dict"][k] for k in self.options["keys"]
                ]
                return NodeMsg.SUCCEEDED
            except KeyError:
                self.logerr(
                    f"One of the key ({self.options['keys']}) is not in dict "
                    f"{str(self.inputs['dict'])}"
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["values"] = []
        self.outputs.reset_updated()
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


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

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set key to an empty string. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["key"] = ""
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("key"):
            try:
                self.outputs["value"] = self.options["dict"][self.inputs["key"]]
                return NodeMsg.SUCCEEDED
            except KeyError:
                self.logerr(
                    f"Key {self.inputs['key']} is not in dict {str(self.options['dict'])}"
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["value"] = None
        self.outputs.reset_updated()
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE


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
    """Get a specific attribute form a python object."""

    def _do_setup(self):
        for child in self.children:
            child.setup()
            # We have a child, so set object to an empty object. We're avoiding an
            # error this way because we know what we're doing, don't use this
            # gratuitously!
            self.inputs["object"] = object()
            self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_tick(self):
        # Tick child (if any) so it can produce its output before we process it
        for child in self.children:
            child.tick()

        if self.inputs.is_updated("object"):
            try:
                self.outputs["attr"] = rgetattr(
                    self.inputs["object"], self.options["attr_name"]
                )
                return NodeMsg.SUCCEEDED
            except AttributeError:
                self.logerr(
                    f"Object {self.inputs['object']} does not have attribute "
                    f"{self.options['attr_name']}"
                )
                return NodeMsg.FAILED
        else:
            if self.options["succeed_on_stale_data"]:
                return NodeMsg.SUCCEEDED
            else:
                self.loginfo("No new data since last tick!")
                return NodeMsg.RUNNING

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        self.outputs["attr"] = None
        self.outputs.reset_updated()
        self._do_setup()
        self.inputs.reset_updated()
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE
