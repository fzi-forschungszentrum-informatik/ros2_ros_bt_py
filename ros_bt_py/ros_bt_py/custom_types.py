# Copyright 2024 FZI Forschungszentrum Informatik
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

from abc import ABC, abstractmethod
import rosidl_runtime_py
import rosidl_runtime_py.utilities


# NOTE These constants serve as docs for supported wrappings and to avoid typos
TYPE_BUILTIN = 'builtin'
DICT_ROS = 'ros'
class TypeWrapper(object):
    """
    This allows to wrap any builtin type to supply additional information,
    like restrictions, suggestions, ...
    """
    def __init__(self, actual_type: type, info=''):
        self.actual_type = actual_type
        self.info = info


class FilePath(object):
    def __init__(self, path=''):
        self.path = path


#NOTE Math types for operation and operand remain in `helpers.py`
# to not cause a breaking change. If there ever is a breaking change for those,
# the new version should be moved here.
from .helpers import MathBinaryOperator, MathUnaryOperator
from .helpers import MathOperandType, MathUnaryOperandType


class FilePath(object):
    def __init__(self, path=""):
        self.path = path


class RosType(ABC):
    # NOTE Subclasses should supply a working default
    @abstractmethod
    def __init__(self, type_str):
        self.type_str = type_str

    # NOTE We can't do the conversion in `__init__`,
    # because jsonpickle bypasses the init function.
    @abstractmethod
    def get_type_obj(self):
        pass


class RosTopicType(RosType):
    def __init__(self, type_str="example_interfaces/msg/Empty"):
        super().__init__(type_str)

    def get_type_obj(self):
        return rosidl_runtime_py.utilities.get_message(self.type_str)


class RosServiceType(RosType):
    def __init__(self, type_str="example_interfaces/srv/Trigger"):
        super().__init__(type_str)

    def get_type_obj(self):
        return rosidl_runtime_py.utilities.get_service(self.type_str)


class RosActionType(RosType):
    def __init__(self, type_str="example_interfaces/action/Fibonacci"):
        super().__init__(type_str)

    def get_type_obj(self):
        return rosidl_runtime_py.utilities.get_action(self.type_str)


# NOTE These should only be used if the service name is specified via options.
# If the name is specified via inputs just use string instead
class RosName(ABC):
    def __init__(self, name=""):
        self.name = name


class RosTopicName(RosName):
    pass


class RosServiceName(RosName):
    pass


class RosActionName(RosName):
    pass
