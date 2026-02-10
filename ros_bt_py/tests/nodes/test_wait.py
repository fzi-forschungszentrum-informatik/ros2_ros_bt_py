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
from typing import Dict
import pytest
import time

from ros_bt_py.nodes.wait import Wait, WaitInput
from ros_bt_py_interfaces.msg import NodeState


@pytest.mark.parametrize(
    "wait_time",
    [
        (1.0),
        (0.05),
    ],
)
class TestWait:

    @pytest.fixture
    def target_node(self, logging_mock, wait_time: float):
        node = Wait(
            options={"seconds_to_wait": wait_time},
            logging_manager=logging_mock,
        )
        return node

    def test_node_success(self, target_node: Wait, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    def test_node_shutdown(self, target_node, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

        target_node.setup()
        assert target_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    def test_node_reset(self, target_node, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        target_node.reset()
        assert target_node.state == NodeState.IDLE

        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1


@pytest.mark.parametrize(
    "wait_time",
    [
        (1.0),
        (0.05),
    ],
)
class TestWaitInput:

    @pytest.fixture
    def target_node(self, logging_mock):
        node = WaitInput(logging_manager=logging_mock)
        return node

    def test_node_success(self, target_node: WaitInput, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    def test_node_shutdown(self, target_node: WaitInput, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        target_node.shutdown()
        assert target_node.state == NodeState.SHUTDOWN

        target_node.setup()
        assert target_node.state == NodeState.IDLE
        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

    def test_node_reset(self, target_node, wait_time: float):
        target_node.setup()
        assert target_node.state == NodeState.IDLE

        target_node.inputs["seconds_to_wait"] = wait_time

        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1

        target_node.reset()
        assert target_node.state == NodeState.IDLE

        start_time = time.time()
        while NodeState.RUNNING == target_node.tick().ok():
            #        assert time.time() - start_time < wait_time
            pass
        end_time = time.time()
        assert NodeState.SUCCEED == target_node.state
        assert end_time - start_time >= wait_time
        assert end_time - start_time < wait_time * 1.1
