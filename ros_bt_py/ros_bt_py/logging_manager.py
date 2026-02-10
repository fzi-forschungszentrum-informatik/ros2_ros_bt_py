# Copyright 2026 FZI Forschungszentrum Informatik
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
import inspect
import os
import re
import uuid

from typing import Callable, Optional
from typeguard import typechecked

from rclpy.impl import rcutils_logger
import rclpy.node

from ros_bt_py.ros_helpers import uuid_to_ros

from ros_bt_py_interfaces.msg import BTLogMessage
from ros_bt_py_interfaces.srv import SetLogLevel


# Extend the rcutils file filter, so this file isn't picked up by logging
rcutils_logger._internal_callers.append(__file__)


@typechecked
class LoggingManager:
    # Log levels are meant to be universal, therefore we assign them at the class level
    _min_log_level: int = BTLogMessage.INFO

    def __init__(
        self,
        ros_node: rclpy.node.Node,
        publish_log_callback: Optional[Callable[[BTLogMessage], None]] = None,
    ):
        self._ros_node = ros_node
        self.publish_log_callback = publish_log_callback
        self.tree_id: Optional[uuid.UUID] = None
        self.tree_name = ""

    def set_tree_info(self, tree_id: uuid.UUID, tree_name: str):
        self.tree_id = tree_id
        self.tree_name = tree_name

    @classmethod
    def set_min_log_level(
        cls,
        set_level_req: SetLogLevel.Request,
        set_level_res: SetLogLevel.Response,
    ):
        cls._min_log_level = set_level_req.min_log_level
        return set_level_res

    @classmethod
    def log_level_inactive(cls, log_level: int) -> bool:
        return cls._min_log_level > log_level

    @staticmethod
    def get_ros_log_name(
        uuid: uuid.UUID,
        name: str,
    ) -> str:
        cleaned_name = re.sub(r"\W+", "_", name)
        return f"{cleaned_name}({uuid})"

    def get_ros_logger(
        self,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
    ) -> rcutils_logger.RcutilsLogger:
        ros_logger = self._ros_node.get_logger()
        if self.tree_id is not None:
            ros_logger = ros_logger.get_child(
                self.get_ros_log_name(self.tree_id, self.tree_name)
            )
            if node_id is not None:
                ros_logger = ros_logger.get_child(
                    self.get_ros_log_name(node_id, node_name)
                )
        return ros_logger

    def log(
        self,
        level: int,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=1,
    ):
        log_message = BTLogMessage(
            stamp=self._ros_node.get_clock().now().to_msg(),
            level=level,
            msg=msg,
            tree_name=self.tree_name,
            node_name=node_name,
        )
        if self.tree_id is not None:
            log_message.tree_id = uuid_to_ros(self.tree_id)
        if node_id is not None:
            log_message.node_id = uuid_to_ros(node_id)

        stack = inspect.stack()
        print(stack)
        if stacklevel < 0 or stacklevel >= len(stack):
            frame = stack[-1]
        else:
            frame = stack[stacklevel]

        log_message.file = os.path.abspath(frame.filename)
        log_message.function = frame.function
        log_message.line = frame.lineno
        print(log_message)

        if self.publish_log_callback is not None:
            self.publish_log_callback(log_message)

    def debug(
        self,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=2,
        internal=False,
    ):
        if self.log_level_inactive(BTLogMessage.DEBUG):
            return
        self.get_ros_logger(node_id, node_name).debug(msg)
        if not internal:
            self.log(
                level=BTLogMessage.DEBUG,
                msg=msg,
                node_id=node_id,
                node_name=node_name,
                stacklevel=stacklevel,
            )

    def info(
        self,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=2,
        internal=False,
    ):
        if self.log_level_inactive(BTLogMessage.INFO):
            return
        self.get_ros_logger(node_id, node_name).info(msg)
        if not internal:
            self.log(
                level=BTLogMessage.INFO,
                msg=msg,
                node_id=node_id,
                node_name=node_name,
                stacklevel=stacklevel,
            )

    def warn(
        self,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=2,
        internal=False,
    ):
        if self.log_level_inactive(BTLogMessage.WARN):
            return
        self.get_ros_logger(node_id, node_name).warn(msg)
        if not internal:
            self.log(
                level=BTLogMessage.WARN,
                msg=msg,
                node_id=node_id,
                node_name=node_name,
                stacklevel=stacklevel,
            )

    def error(
        self,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=2,
        internal=False,
    ):
        if self.log_level_inactive(BTLogMessage.ERROR):
            return
        self.get_ros_logger(node_id, node_name).error(msg)
        if not internal:
            self.log(
                level=BTLogMessage.ERROR,
                msg=msg,
                node_id=node_id,
                node_name=node_name,
                stacklevel=stacklevel,
            )

    def fatal(
        self,
        msg: str,
        node_id: Optional[uuid.UUID] = None,
        node_name: str = "",
        stacklevel=2,
        internal=False,
    ):
        # Although the current system doesn't allow for disabling FATAL
        #   we still include the check to future proof the system.
        if self.log_level_inactive(BTLogMessage.FATAL):
            return
        self.get_ros_logger(node_id, node_name).fatal(msg)
        if not internal:
            self.log(
                level=BTLogMessage.FATAL,
                msg=msg,
                node_id=node_id,
                node_name=node_name,
                stacklevel=stacklevel,
            )
