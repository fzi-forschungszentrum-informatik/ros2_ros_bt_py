# Copyright 2023 FZI Forschungszentrum Informatik

from ros_bt_py_interfaces.msg import Node as NodeMsg
from ros_bt_py_interfaces.msg import UtilityBounds
from ros_bt_py.exceptions import BehaviorTreeException

from ros_bt_py.node import Leaf, define_bt_node
from ros_bt_py.node_config import NodeConfig, OptionRef


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "param_type": type,
            "default_value": OptionRef("param_type"),
            "param_name": str,
        },
        inputs={},
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamOption(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamOption node {self.name} does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.options["param_name"]):
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.options["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.options["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={
            "param_type": type,
            "param_name": str,
        },
        inputs={
            "default_value": OptionRef("param_type"),
        },
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamOptionDefaultInput(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamOptionDefaultInput node {self.name} "
                "does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.options["param_name"]):
            self.outputs["param"] = self.inputs["default_value"]
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.options["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.inputs["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()


@define_bt_node(
    NodeConfig(
        version="0.1.0",
        options={"param_type": type, "default_value": OptionRef("param_type")},
        inputs={"param_name": str},
        outputs={"param": OptionRef("param_type")},
        max_children=0,
    )
)
class RosParamInput(Leaf):
    """Read a parameter from the ROS parameter server."""

    def _do_setup(self):
        if not self.has_ros_node:
            error_msg = (
                f"RosParamInput node {self.name} does not have ROS node reference!"
            )
            self.logerr(error_msg)
            raise BehaviorTreeException(error_msg)

    def _do_tick(self):
        if not self.ros_node.has_parameter(self.inputs["param_name"]):
            self.outputs["param"] = self.options["default_value"]
            return NodeMsg.FAILURE
        else:
            param = self.ros_node.get_parameter(self.inputs["param_name"])
            param_value = param.value
            if param_value is None:
                self.outputs["param"] = self.options["default_value"]
                return NodeMsg.SUCCEEDED
            else:
                if isinstance(param_value, self.options["param_type"]):
                    self.outputs["param"] = param_value
                    return NodeMsg.SUCCEEDED
                else:
                    return NodeMsg.FAILED

    def _do_shutdown(self):
        pass

    def _do_reset(self):
        return NodeMsg.IDLE

    def _do_untick(self):
        return NodeMsg.IDLE

    def _do_calculate_utility(self):
        return UtilityBounds()
