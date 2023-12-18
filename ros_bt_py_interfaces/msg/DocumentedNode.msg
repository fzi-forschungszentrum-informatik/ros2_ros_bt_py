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
# The python module the node is from
string module
# The python class name of the node
string node_class
# The version of this node
string version

# The maximum amount of children this node supports, where -1 means
# infinite children
int32 max_children

## These are run-time parameters
# Each node name must be unique in its tree. This does not count for
# subtrees, as they're handled separately.
string name
# Order is important for child_names!
string[] child_names
NodeData[] options
NodeData[] inputs
NodeData[] outputs

# Documentation
string doc

# Tags
string[] tags

# Should be one of the constants below
string state

string UNINITIALIZED = UNINITIALIZED
string IDLE = IDLE
string RUNNING = RUNNING
string SUCCEEDED = SUCCEEDED
string SUCCEED = SUCCEEDED
string SUCCESS = SUCCEEDED
string FAILED = FAILED
string FAIL = FAILED
string FAILURE = FAILED
string BROKEN = BROKEN
string PAUSED = PAUSED
string SHUTDOWN = SHUTDOWN
string DEBUG_PRE_TICK = DEBUG_PRE_TICK
string DEBUG_TICK = DEBUG_TICK
string DEBUG_POST_TICK = DEBUG_POST_TICK
