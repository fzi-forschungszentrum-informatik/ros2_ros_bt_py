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

uint8 UNDEFINED_TYPE=0
uint8 BOOL_TYPE=1
uint8 INT_TYPE=2
uint8 FLOAT_TYPE=3
uint8 STRING_TYPE=4
uint8 LIST_TYPE=5
uint8 DICT_TYPE=6
uint8 BYTES_TYPE=7
uint8 PATH_TYPE=8
uint8 GENERIC_TYPE=9
uint8 ROS_INTERFACE_NAME=10
uint8 ROS_INTERFACE_TYPE=11
uint8 ROS_INTERFACE_VALUE=12
uint8 REFERENCE_TYPE=13
uint8 REFERENCE_LIST_TYPE=14
uint8 REFERENCE_DICT_TYPE=15


# One of the constants given above
uint8 type_identifier

# If the current type has a specific value type,
# its identifier is given here as one of the constants as above.
# This is given as a list to enable nesting.
# We use uint16 here instead of uint8 to avoid weird type coersion to base64 strings in roslibjs
uint16[] value_type_identifier

bool allow_dynamic
bool allow_static
bool is_static

string[] serialized_value_options

# This is only set for numeric types (int and float) but is passed as a string
#   to avoid issues with limited precision
string min_value
string max_value

uint64 string_max_length

# Those are given as list to support nested structures
uint64[] iterable_max_length
# If strict_length, the maximum has to be matched exactly
bool[] iterable_strict_length

# If this type references another IO field, this contains the key for that field
string reference_target

uint8 ROS_UNDEFINED=0
uint8 ROS_TOPIC=1
uint8 ROS_SERVICE=2
uint8 ROS_ACTION=3
uint8 ROS_COMPONENT=4

uint8 ros_interface_kind

# If this type holds a ROS message, this is set to its interface name (e.g. std_msg/msg/Int64)
string ros_msg_type

# Can be used to tie multiple fields together, e.g. a Topic name its type
uint8 interface_id
