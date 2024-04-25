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
import inspect

import rclpy.logging

from ros_bt_py.exceptions import BehaviorTreeException


class LoggerLevel(object):
    """Data class containing a logging level."""

    def __init__(self, logger_level=rclpy.logging.LoggingSeverity.INFO):
        """Initialize a new LoggerLevel class."""
        self.logger_level = logger_level


class EnumValue(object):
    """Data class containing an enum value."""

    def __init__(self, enum_value=""):
        """Initialize a new EnumValue class."""
        self.enum_value = enum_value


def get_message_constant_fields(message_class):
    """Return all constant fields of a message as a list."""
    if inspect.isclass(message_class):
        # This is highly dependend on the ROS message class generation.
        members = [
            attr
            for attr in dir(message_class.__class__)
            if not attr.startswith("_")
            and not callable(getattr(message_class.__class__, attr))
        ]

        return members
    else:
        raise BehaviorTreeException(f"{message_class} is not a ROS Message")
