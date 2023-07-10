# The time of message generation
builtin_interfaces/Time stamp
# The python module the node is from
string module
# The python class name of the node
string node_class
# The name of the node
string name
# The path from root to this node in the tree
string[] path
# The current execution state for the node from one of the constants below
string state

string PRE_SETUP = PRE_SETUP
string POST_SETUP = POST_SETUP
string PRE_TICK = PRE_TICK
string POST_TICK = POST_TICK
string PRE_UNTICK = PRE_UNTICK
string POST_UNTICK = POST_UNTICK
string PRE_RESET = PRE_RESET
string POST_RESET = POST_RESET
string PRE_SHUTDOWN = PRE_SHUTDOWN
string POST_SHUTDOWN = POST_SHUTDOWN
