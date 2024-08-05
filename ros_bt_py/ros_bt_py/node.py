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
"""Module defining the Node class and helper functions representing a node in the behavior tree."""

from contextlib import contextmanager
from copy import deepcopy

import importlib
import re
from typing import Type, List, Dict, Optional

import rclpy
import rclpy.logging
from rclpy.node import Node as ROSNode

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import NodeData as NodeDataMsg
from ros_bt_py_interfaces.msg import NodeDataLocation, NodeDataWiring
from ros_bt_py_interfaces.msg import Tree
from ros_bt_py_interfaces.msg import UtilityBounds

from ros_bt_py.debug_manager import DebugManager
from ros_bt_py.subtree_manager import SubtreeManager
from ros_bt_py.exceptions import BehaviorTreeException, NodeStateError, NodeConfigError
from ros_bt_py.node_data import NodeData, NodeDataMap
from ros_bt_py.node_config import NodeConfig, OptionRef
from ros_bt_py.helpers import get_default_value, json_decode


def _check_node_data_match(
    node_config: Dict[str, Type], node_data: List[NodeDataMsg]
) -> bool:
    for data in node_data:
        try:
            value = node_config.get(data.key)
        except KeyError:
            return False
        if value is not data.serialized_type:
            return False
    return True


def _set_data_port(
    data_map: NodeDataMap, configuration: List, type: str, permissive=False
) -> None:
    try:
        for msg in configuration:
            try:
                data_map[msg.key] = json_decode(msg.serialized_value)
            except KeyError as exc:
                rclpy.logging.get_logger(data_map.name).warn(
                    f"Could not set a non existing {type} {str(exc)}"
                )
            except AttributeError as exc:
                if permissive:
                    data_map[msg.key] = None
                else:
                    raise AttributeError(
                        f"AttributeError, maybe a ROS Message definition changed. {str(exc)}"
                    ) from exc
    except ValueError as exc:
        raise BehaviorTreeException(
            f"Failed to instantiate node from message: {str(exc)}"
        ) from exc


def _connect_wirings(data_wirings: List, type: str) -> Dict:
    connected_wirings = {}
    for wiring in data_wirings:
        if wiring.source.data_kind == type:
            if wiring.source.node_name in connected_wirings:
                connected_wirings[wiring.source.node_name].append(
                    wiring.source.data_key
                )
            else:
                connected_wirings[wiring.source.node_name] = [wiring.source.data_key]
        elif wiring.target.data_kind == type:
            if wiring.target.node_name in connected_wirings:
                connected_wirings[wiring.target.node_name].append(
                    wiring.target.data_key
                )
            else:
                connected_wirings[wiring.target.node_name] = [wiring.target.data_key]
    return connected_wirings


def _required(meth):
    """
    Mark a method as required.

    Not using :module:`abc` here because a subclass with missing
    methods could still be instantiated and used, just not as part of
    a BT.
    """
    meth._required = True
    return meth


def define_bt_node(node_config):
    """
    Provide information about this Node's interface.

    Every class that derives, directly or indirectly, from :class:`Node`,
    must be decorated with this!

    :param NodeConfig node_config:

    This describes your Node's interface. All inputs, outputs and
    options defined here are automatically registered with your
    class. You should not need to register anything manually!
    """

    def inner_dec(node_class):
        if not issubclass(node_class, Node):
            rclpy.logging.get_logger(node_class.__name__).error(
                f"Class {node_class.__name__} is not a subclass of Node, "
                f"cannot apply define_bt_node decorator!"
            )
            raise TypeError()
        # Merge supplied node config with those of base classes
        for base in node_class.__bases__:
            if hasattr(base, "_node_config") and base._node_config:
                node_config.extend(base._node_config)
        node_class._node_config = node_config

        # Find unimplemented required methods
        missing_methods = set()
        for member in dir(node_class):
            if getattr(getattr(node_class, member, None), "_required", False):
                missing_methods.add(member)

        if missing_methods:
            # Don't register the class if it doesn't implement all required
            # methods
            rclpy.logging.get_logger(node_class.__name__).info(
                f"Assigned NodeData to class {node_class.__name__}, but did not register "
                f"the class because it does not implement all required methods. "
                f"Missing methods: {str(missing_methods)}",
            )
            return node_class

        if node_class.__module__ not in Node.node_classes:
            Node.node_classes[node_class.__module__] = {
                node_class.__name__: [node_class]
            }
        else:
            if node_class.__name__ not in Node.node_classes[node_class.__module__]:
                Node.node_classes[node_class.__module__][node_class.__name__] = [
                    node_class
                ]
            else:

                def __check_dict_equiv(
                    dict1: Dict[str, Type], dict2: Dict[str, Type]
                ) -> bool:
                    if not len(dict1) == len(dict2):
                        return False

                    for key, value in dict1.items():
                        if key not in dict2:
                            return False
                        if not dict2[key] == value:
                            return False
                    return True

                already_available_node_classes = Node.node_classes[
                    node_class.__module__
                ][node_class.__name__]
                candidates = list(
                    filter(
                        lambda node_class_candidate: __check_dict_equiv(
                            node_class_candidate._node_config.inputs, node_config.inputs
                        )
                        and __check_dict_equiv(
                            node_class_candidate._node_config.outputs,
                            node_config.outputs,
                        )
                        and __check_dict_equiv(
                            node_class_candidate._node_config.options,
                            node_config.options,
                        ),
                        already_available_node_classes,
                    )
                )
                if len(candidates) < 1:
                    Node.node_classes[node_class.__module__][
                        node_class.__name__
                    ].append(node_class)
                else:
                    rclpy.logging.get_logger(node_class.__name__).error(
                        "Node class is already registered with this config!"
                    )
        return node_class

    return inner_dec


class NodeMeta(type):
    """
    Override the __doc__ property to add a list of BT params.

    (inputs, outputs and options) to every node class.
    """

    def __new__(cls, name, bases, attrs):
        """Add doc attribute to the new NodeMeta class."""
        attrs["_doc"] = attrs.get("__doc__", "")
        return super(NodeMeta, cls).__new__(cls, name, bases, attrs)

    @property
    def __doc__(self):
        """Generate documentation depending on the node configuration."""
        if (
            hasattr(self, "_node_config")
            and self._node_config is not None
            and (
                self._node_config.inputs
                or self._node_config.outputs
                or self._node_config.options
            )
        ):
            # Build table of inputs, outputs and options
            # Start with two newlines to separate from the original docstring
            param_table = ["\n\n" "**Behavior Tree I/O keys**\n\n"]
            if self._node_config.options:
                param_table.append("*Options*\n\n")
                for option_key in self._node_config.options:
                    param_table.append(
                        f"* {option_key}: :class:"
                        f"`{self._node_config.options[option_key].__name__}`\n"
                    )
                param_table.append("\n")
            if self._node_config.inputs:
                param_table.append("*Inputs*\n\n")
                for input_key in self._node_config.inputs:
                    if isinstance(self._node_config.inputs[input_key], OptionRef):
                        param_table.append(
                            f"* {input_key}: ``{str(self._node_config.inputs[input_key])}``\n"
                        )
                    else:
                        param_table.append(
                            f"* {input_key}: :class:"
                            f"`{self._node_config.inputs[input_key].__name__}`\n"
                        )
                param_table.append("\n")
            if self._node_config.outputs:
                param_table.append("*Outputs*\n\n")
                for output_key in self._node_config.outputs:
                    if isinstance(self._node_config.outputs[output_key], OptionRef):
                        param_table.append(
                            f"* {output_key}: ``{str(self._node_config.outputs[output_key])}``\n"
                        )
                    else:
                        param_table.append(
                            f"* {output_key}: :class:"
                            f"`{self._node_config.outputs[output_key].__name__}`\n"
                        )
                param_table.append("\n")

            return self._doc + "".join(param_table)
        else:
            return self._doc


class Node(object):
    """
    Base class for Behavior Tree nodes.

    Each node has a set of inputs, outputs and options. At every tick
    (usually somewhere between 10 and 30 times a second),
    :meth:`tick` is called with the appropriate data.

    Nodes in a behavior Tree can be roughly divided into two classes,
    with two sub-classes each:

    Leaf Nodes
      These do not have any children and can take one of two forms:
      *Predicates* and *Behaviors*. *Predicates* check a condition and instantly
      return `SUCCEEDED` or `FAILED`. *Behaviors* are more involved and may
      return `RUNNING`, but should be interruptible (see :meth:`untick`).

    Inner Nodes
      These too come in two flavors: *Combiners* and *Decorators*. *Combiners*
      have multiple children and decide which of those children to run (and in
      what fashion) based on some criteria. *Decorators* however have only a
      single child and work with that child's result - for instance, a *Decorator*
      could invert `FAILED` into `SUCCEEDED`.
    """

    __metaclass__ = NodeMeta

    @contextmanager
    def _dummy_report_state(self):
        self.logdebug("Reporting state up without debug manager")
        yield

    @contextmanager
    def _dummy_report_tick(self):
        self.logdebug("Ticking without debug manager")
        yield

    node_classes = {}
    _node_config = None
    permissive = False

    def __init__(
        self,
        options: Optional[Dict] = None,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        name: Optional[str] = None,
        ros_node: Optional[ROSNode] = None,
        succeed_always: bool = False,
        simulate_tick: bool = False,
    ):
        """
        Prepare class members.

        After this finishes, the Node is *not* ready to run. You still
        need to do your own initialization in :meth:`_do_setup`.

        :param succeed_always: If checked the node will return successful after every tick.

        :param simulate_tick: If true, the node should not change the environment,
        eg. by calling external services. These calls should be omitted.

        :param dict options: Map from option names to option values. Use these for configuring
        your node, do not provide a custom `__init()__` method!

        :param debug_manager: Debug manager used to debug a behavior tree.

        :param name: Name of the node - defaults to None, in which case the node name
        is set to the name of its class.

        :raises: NodeConfigError

        If anything is wrong with the node configuration defined via
        :function:`ros_bt_py.node.define_bt_node`
        """
        if name is not None:
            self.name = name
        else:
            self.name = type(self).__name__
        # Only used to make finding the root of the tree easier
        self.parent = None
        self._state = NodeMsg.UNINITIALIZED
        self.children = []

        self.subscriptions = []
        self.subscribers = []

        self._ros_node: Optional[ROSNode] = ros_node
        self.debug_manager = debug_manager
        self.subtree_manager = subtree_manager

        self.succeed_always = succeed_always
        self.simulate_tick = simulate_tick

        if not self._node_config:
            raise NodeConfigError("Missing node_config, cannot initialize!")

        # Copy the class NodeConfig so we can mess with it (but we
        # only should in very rare cases!)
        self.node_config = deepcopy(self._node_config)

        self.options = NodeDataMap(name="options")
        self._register_node_data(
            source_map=self.node_config.options,
            target_map=self.options,
            values=options,
            permissive=self.permissive,
        )

        # Warn about unset options, ignore missing optional_options
        unset_option_keys = [
            key for key in self.options if options is None or key not in options
        ]
        if unset_option_keys:
            optional_keys = []
            for key in unset_option_keys:
                if key in self.node_config.optional_options:
                    optional_keys.append(key)
            if unset_option_keys == optional_keys:
                rclpy.logging.get_logger(self.name).warn(
                    f"missing optional keys: {optional_keys}"
                )
            else:
                raise ValueError(f"Missing options: {str(unset_option_keys)}")

        # Warn about extra options
        if options is not None:
            extra_option_keys = [key for key in options if key not in self.options]
            if extra_option_keys:
                raise ValueError(f"Extra options: {str(extra_option_keys)}")

        self.inputs = NodeDataMap(name="inputs")
        self._register_node_data(
            source_map=self.node_config.inputs, target_map=self.inputs
        )

        self.outputs = NodeDataMap(name="outputs")
        self._register_node_data(
            source_map=self.node_config.outputs, target_map=self.outputs
        )

        # Don't setup automatically - nodes should be available as pure data
        # containers before the user decides to call setup() themselves!

    @property
    def state(self) -> str:
        """State of the node."""
        return self._state

    @state.setter
    def state(self, new_state: str):
        self._state = new_state

    @property
    def has_ros_node(self) -> bool:
        available = self._ros_node is not None
        return available

    @property
    def ros_node(self) -> ROSNode:
        """
        Return the associated ROS node instance.

        If no instance is present an
        """
        if self._ros_node is not None:
            return self._ros_node
        else:
            error_msg = f"{self.name} nodes not have ROS node reference!"
            self.logerr(error_msg)
            raise RuntimeError(error_msg)

    @ros_node.setter
    def ros_node(self, new_ros_node: ROSNode):
        self._ros_node = new_ros_node

    def setup(self) -> str:
        """
        Prepare the node to be ticked for the first time.

        This is called after all the input, output and option values
        have been registered (and in the case of options, populated), so
        you can use those values in your implementation of
        :meth:`_do_setup`

        Sets the state of the node to IDLE.

        :raises: BehaviorTreeException if called when the node is not UNINITIALIZED or SHUTDOWN
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "SETUP")

        with report_state:
            if self.state != NodeMsg.UNINITIALIZED and self.state != NodeMsg.SHUTDOWN:
                raise BehaviorTreeException(
                    "Calling setup() is only allowed in states %s and %s, "
                    "but node %s is in state %s"
                    % (NodeMsg.UNINITIALIZED, NodeMsg.SHUTDOWN, self.name, self.state)
                )
            self.state = self._do_setup()
            if self.state is None:
                self.state = NodeMsg.IDLE
            self.raise_if_in_invalid_state(
                allowed_states=[NodeMsg.IDLE], action_name="setup()"
            )

            self._setup_called = True
        return self.state

    @_required
    def _do_setup(self) -> str:
        """
        Use this to do custom node setup.

        Note that this will be called once, when the tree is first
        started, before the first call of :meth:`tick`.

        :rtype: basestring
        :returns:
          Nothing. If this method doesn't raise an exception, the node state will be
          IDLE afterwards.
        """
        msg = f"Trying to setup a node of type {self.__class__.__name__}"
        "without _do_setup function!"

        self.logfatal(msg)
        return NodeMsg.BROKEN

    def _handle_inputs(self):
        """
        Execute the callbacks registered by :meth:`_wire_input`.

        But only if an input has been updated since the last tick.

        :raises: ValueError
          If any input is unset
        """
        for input_name in self.inputs:
            if not self.inputs.is_updated(input_name):
                self.logdebug("Running tick() with stale data!")
            if self.inputs[input_name] is None:
                # Omit the Error if we declared it to be "okay"
                # This might still not be the best solution but enables some flexibility
                if input_name not in self.node_config.optional_options:
                    raise ValueError(
                        f"Trying to tick a node ({self.name}) with an unset input ({input_name})!"
                    )
        self.inputs.handle_subscriptions()

    def _handle_outputs(self):
        """
        Execute the callbacks registered by :meth:`NodeDataMap.subscribe`: .

        But only if the output has changed during this tick (see where
        the :meth:`NodeDataMap.reset_updated` is called in
        :meth:`tick`)
        """
        self.outputs.handle_subscriptions()

    def tick(self) -> str:
        """
        Handle node on tick action everytime this is called (at ~10-20Hz, usually).

        You should not need to override this method, but instead
        implement :meth:`_do_tick` in your own class.

        :returns:
          The state of the node after ticking - should be `SUCCEEDED`, `FAILED` or `RUNNING`.

        :raises: BehaviorTreeException if a tick is impossible / not allowed
        """
        report_tick = self._dummy_report_tick()
        if self.debug_manager:
            report_tick = self.debug_manager.report_tick(self)

        with report_tick:
            if self.state is NodeMsg.UNINITIALIZED:
                raise BehaviorTreeException("Trying to tick uninitialized node!")

            unset_options = []
            for option_name in self.options:
                if (
                    not self.options.is_updated(option_name)
                    and option_name not in self.node_config.optional_options
                ):
                    unset_options.append(option_name)
            if unset_options:
                msg = f"Trying to tick node with unset options: {str(unset_options)}"
                self.logerr(msg)
                raise BehaviorTreeException(msg)
            self.options.handle_subscriptions()

            # Outputs are updated in the tick. To catch that, we need to reset here.
            self.outputs.reset_updated()

            # Inputs can override options!
            self._handle_inputs()

            self.state = self._do_tick()

            # Inputs are updated by other nodes' outputs, i.e. some time after
            # we use them here. In some cases, inputs might be connected to
            # child outputs (or even our own). If they are, update information
            # is lost, unless it is processed after all child ticks in the same
            # cycle!
            self.inputs.reset_updated()

            self.raise_if_in_invalid_state(
                allowed_states=[
                    NodeMsg.RUNNING,
                    NodeMsg.SUCCEEDED,
                    NodeMsg.FAILED,
                    NodeMsg.ASSIGNED,
                    NodeMsg.UNASSIGNED,
                ],
                action_name="tick()",
            )
            self._handle_outputs()

            return self.state

    def raise_if_in_invalid_state(self, allowed_states: list[str], action_name: str):
        """Raise an error if `self.state` is not in `allowed_states`."""
        if self.state not in allowed_states:
            raise NodeStateError(
                f"Node {self.name} ({type(self).__name__}) was in invalid state "
                f"'{self.state}' after action {action_name}. "
                f"Allowed states: {str(allowed_states)}"
            )

    @_required
    def _do_tick(self) -> str:
        """
        Every Node class must override this.

        This method should **NOT** block, ever, and return one of the
        constants from `Node.Status`.

        :returns:
          One of the constants in :class:`ros_bt_py_msgs.msg.Node`
        """
        msg = f"Ticking a node of type {self.__class__.__name__} without _do_tick function!"
        self.logfatal(msg)
        return NodeMsg.BROKEN

    def untick(self) -> str:
        """
        Signal a node that it should stop any background tasks.

        A new tick has started and this node has **not** been ticked.
        The node's state should be `IDLE` after calling this.

        The node's outputs' `updated` flags are also reset!

        A class inheriting from :class:`Node` should override :meth:`_do_untick` instead of this!

        :raises: BehaviorTreeException

        When trying to untick a node that has not been initialized yet
        by running `setup()`.

        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "UNTICK")

        with report_state:
            if self.state is NodeMsg.UNINITIALIZED:
                raise BehaviorTreeException("Trying to untick uninitialized node!")
            self.state = self._do_untick()
            self.raise_if_in_invalid_state(
                allowed_states=[NodeMsg.IDLE, NodeMsg.PAUSED], action_name="untick()"
            )

            self.outputs.reset_updated()
            return self.state

    @_required
    def _do_untick(self) -> str:
        """
        Abstract method used to implement the actual untick operations.

        This is called by :meth:`untick` - override it!

        After executing this method, your node should:

        1. Be in the IDLE or PAUSED state, unless an error happened
        2. Not execute any of its behavior in the background
        3. Be ready to resume on the next call of :meth:`tick`
        """
        msg = f"Unticking a node of type {self.__class__.__name__} without _do_untick function!"
        self.logfatal(msg)
        return NodeMsg.BROKEN

    def reset(self) -> str:
        """
        Reset a node completly.

        Whereas :meth:`untick` / :meth:`_do_untick` only pauses
        execution, ready to be resumed, :meth:`reset` means returning
        to the same state the node was in right after calling :meth:`setup`

        :raises: BehaviorTreeException
          When trying to reset a node that hasn't been initialized yet
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "RESET")

        with report_state:
            if self.state is NodeMsg.UNINITIALIZED:
                raise BehaviorTreeException("Trying to reset uninitialized node!")

            if self.state is NodeMsg.SHUTDOWN:
                raise BehaviorTreeException("Trying to reset shutdown node!")

            # Reset input/output reset state and set outputs to None
            # before calling _do_reset() - the node can overwrite the None
            # with more appropriate values if need be.
            self.inputs.reset_updated()

            for output_key in self.outputs:
                self.outputs[output_key] = None
            self.outputs.reset_updated()

            self.state = self._do_reset()
            self.raise_if_in_invalid_state(
                allowed_states=[NodeMsg.IDLE], action_name="reset()"
            )
            return self.state

    @_required
    def _do_reset(self) -> str:
        """
        Abstract method used to implement the reset action.

        After executing this method, your node should:

        1. Be in the IDLE state
        2. Not be doing anything in the background
        3. On the next tick, behave as if it has just been created

        :returns:
          The new state of the node (should be IDLE unless an error happened)
        """
        msg = f"Resetting a node of type {self.__class__.__name__} without _do_reset function!"
        self.logfatal(msg)
        return NodeMsg.BROKEN

    def shutdown(self) -> str:
        """
        Prepare a node for deletion.

        This method just calls :meth:`_do_shutdown`, which any
        subclass must override.

        This gives the node a chance to clean up any resources it might
        be holding before getting deleted.

        :meth:`_do_shutdown` will not be called if the node has not been initialized yet.
        """
        report_state = self._dummy_report_state()
        if self.debug_manager:
            report_state = self.debug_manager.report_state(self, "SHUTDOWN")
        with report_state:
            if self.state == NodeMsg.UNINITIALIZED:
                self.loginfo(
                    "Not calling shutdown method, node has not been initialized yet"
                )
                self.state = NodeMsg.SHUTDOWN
                # Call shutdown on all children - this should only set
                # their state to shutdown
                for child in self.children:
                    child.shutdown()

            self.state = self._do_shutdown()
            if self.state is None:
                self.state = NodeMsg.SHUTDOWN

            for child in self.children:
                child.shutdown()

            unshutdown_children = [
                f"{child.name} ({type(child).__name__}), state: {child.state}"
                for child in self.children
                if child.state != NodeMsg.SHUTDOWN
            ]
            if len(unshutdown_children) > 0:
                self.logwarn(
                    "Not all children are shut down after calling shutdown(). "
                    "List of not-shutdown children and states:\n"
                    f"{unshutdown_children}"
                )
            return self.state

    @_required
    def _do_shutdown(self) -> str:
        """
        Abstract method implementing the shutdown action.

        Implement this in your node class and release any resources you
        might be holding (file pointers, ROS topic subscriptions etc.)
        """
        msg = f"Shutting down a node of type {self.__class__.__name__}"
        "without _do_shutdown function!"

        self.logfatal(msg)
        return NodeMsg.BROKEN

    def calculate_utility(self) -> UtilityBounds:
        """
        Calculate the utility bounds for this node.

        Unlike the other node functions, there is a default
        implementation for the corresponding method,
        :meth:`Node._do_calculate_utility()`.

        However, in order to get meaningful results, one should take
        care to use as many nodes as possible that provide their own
        implementation, since the default reports that there is no
        cost for execution.

        """
        return self._do_calculate_utility()

    def _do_calculate_utility(self) -> UtilityBounds:
        """
        Calculate utility values. This is a default implementation.

        :returns:

        A :class:`ros_bt_py_msgs.msg.UtilityBounds` message with
        `can_execute` set to `True`, all bounds set to 0.0 and all of
        the `has_bound` members set to `True`.

        That is, any node that does not override this method is
        considered to execute at no cost at all.

        """
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )

    def get_child_index(self, child_name: str) -> Optional[int]:
        """
        Get the index in the `children` array of the child with the given name.

        This is useful if you want to replace a child with another node.

        :returns:

        An integer index if a child with the given name exists, `None`
        if there's no such child

        """
        try:
            return [child.name for child in self.children].index(child_name)
        except ValueError:
            return None

    def add_child(self, child: "Node", at_index=None) -> "Node":
        """
        Add a child to this node at the given index.

        :raises: BehaviorTreeException, KeyError

        `BehaviorTreeException` if the number of children after the add operation
        would exceed the maximum number of children, `KeyError` if a
        child of the same name already exists
        """
        if (
            self.node_config.max_children is not None
            and len(self.children) == self.node_config.max_children
        ):
            error_msg = (
                "Trying to add child when maximum number of "
                "children (%d) is already present" % self.node_config.max_children
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

        if child.name in (child1.name for child1 in self.children):
            raise KeyError(f'Already have a child with name "{child.name}"')
        if at_index is None:
            at_index = len(self.children)

        if at_index < 0:
            at_index += len(self.children) + 1
        # Use array slicing to efficiently insert child at the correct position
        # (the value we assign needs to be a list for this to work)
        self.children[at_index:at_index] = [child]
        child.parent = self

        # return self to allow chaining of addChild calls
        return self

    def remove_child(self, child_name: str) -> "Node":
        """
        Remove the child with the given name and return it.

        :param basestring child_name: The name of the child to remove

        :rtype: Node
        :returns: The child that was just removed
        :raises: KeyError if no child with that name exists
        """
        child_index = self.get_child_index(child_name)
        if child_index is None:
            raise KeyError(f'Node {self.name} has no child named "{child_name}"')

        tmp = self.children[child_index]
        del self.children[child_index]
        tmp.parent = None
        return tmp

    @staticmethod
    def _find_option_refs(
        source_map: Dict[str, type],
        target_map: NodeDataMap,
        values=None,
        permissive=False,
    ) -> None:
        for key, data_type in {
            k: v for (k, v) in source_map.items() if not isinstance(v, OptionRef)
        }.items():
            if key in target_map:
                raise NodeConfigError(f"Duplicate data name: {key}")
            target_map.add(key, NodeData(data_type=data_type))
            if values is not None and key in values:
                try:
                    target_map[key] = values[key]
                except TypeError as e:
                    if permissive:
                        if data_type == type:
                            target_map[key] = int
                            # if data_type is a type check if a OptionRef exists
                            for key_opt, value_opt in source_map.items():
                                if isinstance(value_opt, OptionRef):
                                    if value_opt.option_key == key:
                                        values[key_opt] = 0
                                        if key_opt in target_map:
                                            # overwrite already set target_map
                                            target_map[key_opt] = 0
                        else:
                            raise e
                    else:
                        raise e

    def _register_node_data(
        self,
        source_map: Dict[str, type],
        target_map: NodeDataMap,
        values=None,
        permissive=False,
    ) -> None:
        """
        Register a number of typed :class:`NodeData` in the given map.

        Note that when using :class:`OptionRef`, the option keys
        referenced by any :class:`OptionRef` objects must exist and be
        populated!

        :param dict(str, type) source_map: a dictionary mapping from data keys to data types,
        i.e. ``{ 'a_string' : str, 'an_int' : int }``

        :param NodeDataMap target_map:
        The :class:`NodeDataMap` to add :class:`NodeData` values to

        :param dict(str, value) values:
        An optional dictionary containing the values for the NodeData

        :raises: NodeConfigError in any of the following cases:
          * If any of the keys in `source_map` already exist in `target_map`
          * If an OptionRef value is passed, but `allow_ref` is `False`
          * If an OptionRef references an option value that has not been set or
            does not exist
          * If an OptionRef references an option value that does not hold a `type`

        """
        # Find the values that are not OptionRefs first
        self._find_option_refs(
            source_map=source_map,
            target_map=target_map,
            permissive=permissive,
            values=values,
        )

        # Now process OptionRefs
        for key, data_type in {
            k: v for (k, v) in source_map.items() if isinstance(v, OptionRef)
        }.items():
            if key in target_map:
                raise NodeConfigError(f"Duplicate {target_map.name} data name: {key}")

            if data_type.option_key not in self.options:
                raise NodeConfigError(
                    'OptionRef for %s key "%s" references invalid '
                    'option key "%s"' % (target_map.name, key, data_type.option_key)
                )
            if not self.options.is_updated(data_type.option_key):
                self.loginfo(str(self.options))
                raise NodeConfigError(
                    'OptionRef for %s key "%s" references unwritten '
                    'option key "%s"' % (target_map.name, key, data_type.option_key)
                )
            if not isinstance(self.options[data_type.option_key], type):
                raise NodeConfigError(
                    'OptionRef for %s key "%s" references option key '
                    '"%s" that does not contain a type!'
                    % (target_map.name, key, data_type.option_key)
                )
            target_map.add(key, NodeData(data_type=self.options[data_type.option_key]))
            if values is not None and key in values:
                try:
                    target_map[key] = values[key]
                except AttributeError as e:
                    if permissive:
                        if (
                            type(values[key]).__slots__ is not None
                            and type(values[key])._slot_types is not None
                        ):
                            fixed_new_value = type(values[key])()

                            for i, slot in enumerate(type(values[key]).__slots__):
                                setattr(
                                    fixed_new_value,
                                    slot,
                                    getattr(
                                        values[key],
                                        slot,
                                        get_default_value(
                                            type(getattr(fixed_new_value, slot, int)),
                                            ros=True,
                                        ),
                                    ),
                                )
                            target_map[key] = fixed_new_value
                        else:
                            raise AttributeError(
                                f"AttributeError, maybe a ROS Message definition changed. {str(e)}"
                            )
                    else:
                        raise AttributeError(
                            f"AttributeError, maybe a ROS Message definition changed. {str(e)}"
                        )

    def __repr__(self):
        """Create a string representation of the node class."""
        return (
            "%s(options=%r, name=%r), parent_name:%r, state:%r, inputs:%r, outputs:%r,"
            " children:%r"
            % (
                type(self).__name__,
                {key: self.options[key] for key in self.options},
                self.name,
                self.parent.name if self.parent else "",
                self.state,
                self.inputs,
                self.outputs,
                self.children,
            )
        )

    def __eq__(self, other):
        """Check if all attributes of a node are equal."""
        return (
            self.name == other.name
            and self.parent == other.parent
            and self.state == other.state
            and type(self).__module__ == type(other).__module__
            and type(self).__name__ == type(other).__name__
            and self.options == other.options
            and self.inputs == other.inputs
            and self.outputs == other.outputs
            and self.children == other.children
        )

    def __ne__(self, other):
        """Check if two nodes have a single differing attribute."""
        return not self == other

    def get_data_map(self, data_kind):
        """
        Return one of our NodeDataMaps by string name.

        :param basestring data_kind:
          One of the constants in :class:`ros_bt_py_msgs.msg.NodeDataLocation`

        :rtype: NodeDataMap
        """
        if data_kind == NodeDataLocation.INPUT_DATA:
            return self.inputs
        if data_kind == NodeDataLocation.OUTPUT_DATA:
            return self.outputs
        if data_kind == NodeDataLocation.OPTION_DATA:
            return self.options

        raise KeyError(
            f"{data_kind} is not a valid value to pass to Node.get_data_map()!"
        )

    # Logging methods - these just use the ROS logging framework, but add the
    # name and type of the node so it's easier to trace errors.

    def logdebug(self, message) -> None:
        """
        Wrap call to :func:rclpy.logger.get_logger(...).debug.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).debug(f"{message}")

    def loginfo(self, message) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).info.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).info(f"{message}")

    def logwarn(self, message) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).warn.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).warn(f"{message}")

    def logerr(self, message) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).error.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).error(f"{message}")

    def logfatal(self, message) -> None:
        """
        Wrap call to :func:rclpy.logging.get_logger(...).fatal.

        Adds this node's name and type to the given message
        """
        rclpy.logging.get_logger(self.name).fatal(f"{message}")

    @classmethod
    def from_msg(
        cls,
        msg: NodeMsg,
        ros_node: ROSNode,
        debug_manager: Optional[DebugManager] = None,
        subtree_manager: Optional[SubtreeManager] = None,
        permissive: bool = False,
    ) -> "Node":
        """
        Construct a Node from the given ROS message.

        This will try to import the requested node class, instantiate it
        and populate its `name`, `options`, `input` and `output` members
        from the ROS message.

        :param ros_bt_py_msgs.msg.Node msg:

        A ROS message describing a node class. The node class must be
        available in the current environment (but does not need to be
        imported before calling this).

        :param debug_manager:

        The debug manager to use for the newly instantiated node class.

        :param permissive:

        Enable permissive behavior.

        :returns:

        An instance of the class named by `msg`, populated with the
        values from `msg`.

        Note that this does *not* include the node's state. Any node
        created by this will be in state UNININITIALIZED.

        :raises:

        BehaviorTreeException if
        node cannot be instantiated.
        """
        if (
            msg.module not in cls.node_classes
            or msg.node_class not in cls.node_classes[msg.module]
        ):
            # If the node class was not available, try to load it
            load_node_module(msg.module)

        # If loading didn't work, abort
        if (
            msg.module not in cls.node_classes
            or msg.node_class not in cls.node_classes[msg.module]
        ):
            raise BehaviorTreeException(
                "Failed to instantiate node from message - node class not available. "
                f"Original message:\n{str(msg)}"
            )

        node_classes: List[Type[Node]] = cls.node_classes[msg.module][msg.node_class]

        node_class: Optional[type] = None
        if len(node_classes) > 1:
            rclpy.logging.get_logger(cls.__name__).warn(f"{msg} - {node_classes}")
            candidates = list(
                filter(
                    lambda node_class_candidate: _check_node_data_match(
                        node_class_candidate._node_config.inputs, msg.inputs
                    )
                    and _check_node_data_match(
                        node_class_candidate._node_config.outputs, msg.outputs
                    )
                    and _check_node_data_match(
                        node_class_candidate._node_config.options, msg.options
                    ),
                    node_classes,
                )
            )
            if len(candidates) < 1:
                raise BehaviorTreeException(
                    "Failed to instantiate node from message - node class not available."
                    f"Original message:\n{str(msg)}"
                )
            if len(candidates) > 1:
                raise BehaviorTreeException(
                    "Failed to instantiate node from message - multiple versions of node class "
                    f"available. Original message:\n {str(msg)}"
                )
            node_class = candidates[0]
        else:
            node_class = node_classes[0]

        # Populate options dict
        options_dict = {}
        try:
            for option in msg.options:
                options_dict[option.key] = json_decode(option.serialized_value)
        except ValueError as exc:
            raise BehaviorTreeException(
                f"Failed to instantiate node from message: {str(exc)}"
            ) from exc

        # Instantiate node - this shouldn't do anything yet, since we don't
        # call setup()
        node_class.permissive = permissive
        if msg.name:
            node_instance = node_class(
                name=msg.name,
                options=options_dict,
                debug_manager=debug_manager,
                subtree_manager=subtree_manager,
                ros_node=ros_node,
            )
        else:
            node_instance = node_class(
                options=options_dict,
                debug_manager=debug_manager,
                subtree_manager=subtree_manager,
                ros_node=ros_node,
            )

        _set_data_port(node_instance.inputs, msg.inputs, "input", permissive)
        _set_data_port(node_instance.outputs, msg.outputs, "output", permissive)

        return node_instance

    def get_children_recursive(self):
        """Return all nodes that are below this node in the parent-child hirachy recursively."""
        yield self
        for child in self.children:
            for child_rec in child.get_children_recursive():
                yield child_rec

    def get_subtree_msg(self):
        """
        Populate a TreeMsg with the subtree rooted at this node.

        This can be used to "shove" a subtree to a different host, by
        using that host's load_tree service.

        The subtree message will have public node data for every piece
        of node data that is wired to a node outside the subtree.

        :returns:

        A tuple consisting of a :class:`ros_bt_py_msgs.msg.Tree`
        message and two lists of
        :class:`ros_bt_py_msgs.msg.NodeDataWiring` messages
        (incoming_connections and outgoing_connections). The latter
        can be used to determine what parameters need to be forwarded
        to / from the remote executor if the subtree is to be executed
        remotely.

        Crucially, the resulting subtree will not be tick-able until all
        the incoming wirings from external_connections have been
        connected.

        However, if the subtree is to be shoved to a different
        executor, it's enough for the incoming wirings to be connected
        in the host tree - this will cause input values to be set and
        sent to the remote executor.

        """
        subtree_name = f"{self.name}_subtree"
        subtree = Tree(
            name=subtree_name,
            root_name=self.name,
            nodes=[node.to_msg() for node in self.get_children_recursive()],
            state=Tree.IDLE,
        )

        node_map = {node.name: node for node in subtree.nodes}
        incoming_connections = []
        outgoing_connections = []
        for node in self.get_children_recursive():
            for sub in node.subscriptions:
                source_node = node_map.get(sub.source.node_name)
                target_node = node_map.get(sub.target.node_name)

                # For subscriptions where source and target are in the subtree,
                # add a wiring.
                if source_node and target_node:
                    subtree.data_wirings.append(
                        NodeDataWiring(source=sub.source, target=sub.target)
                    )
                # In the other cases, add that datum to public_node_data
                elif source_node:
                    subtree.public_node_data.append(sub.source)
                    outgoing_connections.append(sub)
                elif target_node:
                    subtree.public_node_data.append(sub.target)
                    incoming_connections.append(sub)
                else:
                    raise BehaviorTreeException(
                        "Subscription in subtree has source *AND* target "
                        "outside of subtree!"
                    )

            for wiring, _, _ in node.subscribers:
                if wiring.target.node_name not in node_map:
                    subtree.public_node_data.append(wiring.source)
                    outgoing_connections.append(wiring)

        connected_inputs = _connect_wirings(
            subtree.data_wirings, NodeDataLocation.INPUT_DATA
        )
        connected_outputs = _connect_wirings(
            subtree.data_wirings, NodeDataLocation.OUTPUT_DATA
        )

        for node in subtree.nodes:
            for node_input in node.inputs:
                if (
                    node.name not in connected_inputs
                    or node_input.key not in connected_inputs[node.name]
                ):
                    # Input is unconnected, list it as public
                    subtree.public_node_data.append(
                        NodeDataLocation(
                            node_name=node.name,
                            data_kind=NodeDataLocation.INPUT_DATA,
                            data_key=node_input.key,
                        )
                    )
            for node_output in node.outputs:
                if (
                    node.name not in connected_outputs
                    or node_output.key not in connected_outputs[node.name]
                ):
                    # Input is unconnected, list it as public
                    subtree.public_node_data.append(
                        NodeDataLocation(
                            node_name=node.name,
                            data_kind=NodeDataLocation.OUTPUT_DATA,
                            data_key=node_output.key,
                        )
                    )
        return subtree, incoming_connections, outgoing_connections

    def find_node(self, other_name):
        """
        Try to find the node with the given name in the tree.

        This is not a particularly cheap operation, since it ascends
        the tree up to the root and then recursively descends back
        until it finds the node.

        Probably best not to use it in a tick function.

        """
        root = self
        while root.parent is not None:
            root = root.parent

        for node in root.get_children_recursive():
            if node.name == other_name:
                return node

        return None

    def _subscribe(self, wiring, new_cb, expected_type):
        """
        Subscribe to a piece of Nodedata this node has.

        Call this on a node to *subscribe to NodeData **from** that
        node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Defines the source and target of the subscribe operation.
        `wiring.source` must point to a valid data key in this node
        (`self`).  `wiring.target` is not checked here (because it's
        only used to label subscriptions), but it should also point to a
        valid data key of a node in the tree.

        :param cb:

        A callback function that will be called whenever there's an updated
        value for the key (and this node receives a tick)

        :param `type` expected_type:

        The type that the subscriber expects our piece of data to
        have. If it doesn't match the type of the data at the requested
        key, raise a `BehaviorTreeException`.

        :raises:

        KeyError if the requested data location is not in this node, or
        the requested key does not exist in this node.

        BehaviorTreeException if `expected_type` and the actual type of
        the data are incompatible.
        """
        if wiring.source.node_name != self.name:
            raise KeyError(
                f"{self.name}: Trying to subscribe to another node ({wiring.source.node_name})"
            )

        for sub, _, _ in self.subscribers:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    raise BehaviorTreeException("Duplicate subscription!")
                self.logwarn(
                    f"Subscriber {wiring.target.node_name} is subscribing to multiple sources "
                    f"with the same target {wiring.target.data_kind}[{wiring.target.data_key}]"
                )

        source_map = self.get_data_map(wiring.source.data_kind)

        if wiring.source.data_key not in source_map:
            raise KeyError(
                f"Source key {self.name}.{wiring.source.data_kind}[{wiring.source.data_key}] "
                "does not exist!"
            )

        if not issubclass(source_map.get_type(wiring.source.data_key), expected_type):
            raise BehaviorTreeException(
                f"Type of {self.name}.{wiring.source.data_kind}[{wiring.source.data_key}] "
                f"({source_map.get_type(wiring.source.data_key).__name__}) is not compatible with "
                "Type of "
                f"{wiring.target.node_name}.{wiring.target.data_kind}[{wiring.target.data_key}] "
                f"({expected_type})!"
            )

        source_map.subscribe(
            wiring.source.data_key,
            new_cb,
            f"{wiring.target.node_name}.{wiring.target.data_kind}[{wiring.target.data_key}]",
        )
        self.subscribers.append((deepcopy(wiring), new_cb, expected_type))

    def wire_data(self, wiring):
        """
        Wire a piece of Nodedata from another node to this node.

        Call this on a node to *connect it to NodeData from
        **another** node*!

        :param `ros_bt_py_msgs.msg.NodeDataWiring` wiring:

        Indicates both the piece of NodeData we want to subscribe to
        (`wiring.source`) and the data key inside this node we want to
        connect it to (`wiring.target`).

        If we cannot find a node named `wiring.source.node_name`, or the location is
        otherwise invalid, this will raise a KeyError.

        :param `ros_bt_py_msgs.msg.NodeDataLocation` target:
           The position *inside this node* we want to wire *to*.

        :raises:

        KeyError if either the source or target location is invalid.

        BehaviorTreeException if a subscription with the same source and
        target exists already, or if the types of source and target data
        are incompatible.
        """
        if wiring.target.node_name != self.name:
            raise BehaviorTreeException(
                f"Target of wiring ({wiring.target.node_name}) is not this node ({self.name})"
            )

        for sub in self.subscriptions:
            if sub.target == wiring.target:
                if sub.source == wiring.source:
                    raise BehaviorTreeException("Duplicate subscription!")
                # self.logwarn('Subscribing to two different sources for key %s[%s]'
                #              % (wiring.target.data_kind, wiring.target.data_key))
        source_node = self.find_node(wiring.source.node_name)
        if not source_node:
            raise BehaviorTreeException(
                f"Source node {wiring.source.node_name} does not exist or is not connected "
                f"to target node {self.name}"
            )
        try:
            source_map = source_node.get_data_map(wiring.source.data_kind)
        except KeyError as exception:
            raise BehaviorTreeException(str(exception)) from exception

        if wiring.source.data_key not in source_map:
            raise KeyError(
                f"Source key {source_node.name}."
                f"{wiring.source.data_kind}[{wiring.source.data_key}] does not exist!"
            )

        try:
            target_map = self.get_data_map(wiring.target.data_kind)
        except KeyError as exception:
            raise BehaviorTreeException(str(exception)) from exception

        if wiring.target.data_key not in target_map:
            raise KeyError(
                f"Target key {self.name}."
                f"{wiring.target.data_kind}[{wiring.target.data_key}] does not exist!"
            )

        source_node._subscribe(
            wiring,
            target_map.get_callback(wiring.target.data_key),
            target_map.get_type(wiring.target.data_key),
        )

        self.subscriptions.append(deepcopy(wiring))

    def _unsubscribe(self, wiring):
        """
        Unsubscribe from a piece of NodeData this node has.

        Call this to undo a call to `Node._subscribe()'

        :raises:

        KeyError if the requested data location is not in this node, or
        the requested key does not exist in this node.

        """
        if wiring.source.node_name != self.name:
            raise KeyError(
                f"{self.name}: Trying to unsubscribe from another node ({wiring.source.node_name})"
            )
        source_map = self.get_data_map(wiring.source.data_kind)

        if wiring.source.data_key not in source_map:
            raise KeyError(
                f"Source key {self.name}."
                f"{wiring.source.data_kind}[{wiring.source.data_key}] does not exist!"
            )

        for sub_wiring, callback, _ in self.subscribers:
            if wiring.target == sub_wiring.target:
                source_map.unsubscribe(wiring.source.data_key, callback)
        # remove subscriber data from list
        self.subscribers = [
            sub for sub in self.subscribers if sub[0].target != wiring.target
        ]

    def unwire_data(self, wiring):
        """
        Unwire the given wiring.

        This entails finding the source of the wiring, calling its
        :meth:`Node._unsubscribe` method and removing the wiring from
        this node's list of subscriptions.

        :raises: BehaviorTreeException

        If the given wiring's source node cannot be found from this
        node.
        """
        if wiring.target.node_name != self.name:
            raise KeyError(
                f"Target of wiring ({wiring.target.node_name}) is not this node ({self.name})"
            )
        source_node = self.find_node(wiring.source.node_name)

        if wiring not in self.subscriptions:
            # Nothing to do
            return

        if source_node:
            source_node._unsubscribe(wiring)
        self.subscriptions.remove(wiring)

        # If the removed wiring was the last subscription for this
        # datum, set it to None
        if not [
            sub
            for sub in self.subscriptions
            if (
                sub.target.data_kind == wiring.target.data_kind
                and sub.target.data_key == wiring.target.data_key
            )
        ]:
            if wiring.target.data_kind == NodeDataLocation.INPUT_DATA:
                self.inputs[wiring.target.data_key] = None
            elif wiring.target.data_kind == NodeDataLocation.OUTPUT_DATA:
                self.outputs[wiring.target.data_key] = None
            elif wiring.target.data_kind == NodeDataLocation.OPTION_DATA:
                self.options[wiring.target.data_key] = None

    def to_msg(self):
        """
        Populate a ROS message with the information from this Node.

        Round-tripping the result through :meth:`Node.from_msg` should
        yield a working node object, with the caveat that state will not
        be preserved.

        :rtype: ros_bt_py_msgs.msg.Node
        :returns:

        A ROS message that describes the node.
        """
        node_type = type(self)

        return NodeMsg(
            module=node_type.__module__,
            node_class=node_type.__name__,
            version=self.node_config.version,
            name=self.name,
            child_names=[child.name for child in self.children],
            options=[
                NodeDataMsg(
                    key=key,
                    serialized_value=self.options.get_serialized(key),
                    serialized_type=self.options.get_serialized_type(key),
                )
                for key in self.options
            ],
            inputs=[
                NodeDataMsg(
                    key=key,
                    serialized_value=self.inputs.get_serialized(key),
                    serialized_type=self.inputs.get_serialized_type(key),
                )
                for key in self.inputs
            ],
            outputs=[
                NodeDataMsg(
                    key=key,
                    serialized_value=self.outputs.get_serialized(key),
                    serialized_type=self.outputs.get_serialized_type(key),
                )
                for key in self.outputs
            ],
            max_children=(
                self.node_config.max_children
                if self.node_config.max_children is not None
                else -1
            ),
            state=self.state,
        )


def load_node_module(package_name):
    """
    Import the named module at run-time.

    If the module contains any (properly decorated) node classes,
    they will be registered and available to load via the other
    commands in this class.
    """
    try:
        return importlib.import_module(package_name)
    except (ImportError, ValueError) as exc:
        rclpy.logging.get_logger(package_name).error(
            f'Could not load node module "{package_name}": {repr(exc)}',
            throttle_duration_sec=30,
        )
        return None


def increment_name(name):
    """
    If `name` does not already end in a number, add "_2" to it.

    Otherwise, increase the number after the underscore.
    """
    match = re.search("_([0-9]+)$", name)
    prev_number = 1
    if match:
        prev_number = int(match.group(1))
        # remove the entire _$number part from the name
        name = name[: len(name) - len(match.group(0))]

    name += f"_{prev_number + 1}"
    return name


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=1))
class Decorator(Node):
    """
    Base class for Decorator nodes.

    Decorators have exactly one child and somehow modify that child's
    output. Subclasses can add inputs, outputs and options, but never
    change `max_children`.
    """

    def _do_calculate_utility(self):
        """Pass on the utility value of the (only allowed) child."""
        if self.children:
            return self.children[0].calculate_utility()
        return UtilityBounds(
            can_execute=True,
            has_lower_bound_success=True,
            has_upper_bound_success=True,
            has_lower_bound_failure=True,
            has_upper_bound_failure=True,
        )


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=0))
class Leaf(Node):
    """
    Base class for leaf nodes in the tree.

    Leaf nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=None))
class FlowControl(Node):
    """
    Base class for flow control nodes.

    Flow control nodes (mostly Sequence, Fallback and their derivatives)
    can have an unlimited number of children and each have a unique set
    of rules for when to tick which of their children.
    """


@define_bt_node(NodeConfig(options={}, inputs={}, outputs={}, max_children=0))
class IO(Node):
    """
    Base class for IO nodes in the tree.

    IO nodes have no children. Subclasses can define inputs, outputs
    and options, but never change `max_children`.
    """
