# A descriptive name for this tree. At runtime, this is chosen by the
# tree manager node.
string name
# A string pointing to a YAML file to read a tree from, if
# any. Can be a file:// or a package:// string. Please don't build
# infinite loops using this.
string path
string root_name
Node[] nodes
NodeDataWiring[] data_wirings
float64 tick_frequency_hz
# One of the constants below
string state

string IDLE = IDLE
string EDITABLE = EDITABLE
string TICKING = TICKING
string WAITING_FOR_TICK = WAITING_FOR_TICK
string DEBUGGING = DEBUGGING
string STOP_REQUESTED = STOP_REQUESTED
string ERROR = ERROR

# These are accessible when using this tree as a subtree
NodeDataLocation[] public_node_data
