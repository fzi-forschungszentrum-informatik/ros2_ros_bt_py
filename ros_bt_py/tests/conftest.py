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
import pytest
import warnings


class WarnLog(Warning):
    """
    This is used in `warnings.warn` when a test prints an WARN level log message.

    You can use `pytest.warns` to test for them.
    """


class ErrorLog(Warning):
    """
    This is used in `warnings.warn` when a test prints an ERROR level log message.

    You can use `pytest.warns` to test for them.
    """


class TestLoggingManager:

    def debug(self, msg, *args, **kwargs):
        # Disregard DEBUG logs to not clog output
        pass

    def info(self, msg, *args, **kwargs):
        print("INFO log: ", msg, "\nFrom ", args, kwargs)

    def warn(self, msg, *args, **kwargs):
        print("WARNING log: ", msg, "\nFrom ", args, kwargs)
        warnings.warn(message=msg, category=WarnLog)

    def error(self, msg, *args, **kwargs):
        print("ERROR log: ", msg, "\nFrom ", args, kwargs)
        warnings.warn(message=msg, category=ErrorLog)

    def fatal(self, msg, *args, **kwargs):
        print("FATAL log: ", msg, "\nFrom ", args, kwargs)
        # FATAL logs always cause the test to fail. This is intentional.
        assert False, "Log message of level FATAL"


@pytest.fixture
def logging_mock():
    return TestLoggingManager()
