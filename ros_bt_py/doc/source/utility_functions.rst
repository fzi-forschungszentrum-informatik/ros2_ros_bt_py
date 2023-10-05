.. _utility-functions:

Utility Functions
=================

Utility Functions are one of the things that make :code:`ros_bt_py` unique.

Here you'll learn a bit about what they are, what they're good for and
how to work with them.

What?
-----

Utility Functions are used to determine which robot is best suited to
execute any given BT.

In this implementation, their values are represented by the
:class:`ros_bt_py_msgs.msg.UtilityBounds` message, which has the
following attributes:

* ``can_execute``

  Pretty self-explanatory - if a critical resoure (like a ROS Topic or
  Action) for executing a BT node is missing, this is ``False``.

* ``has_upper_bound_success` and `upper_bound_success``
* ``has_lower_bound_success` and `lower_bound_success``
* ``has_upper_bound_failure` and `upper_bound_failure``
* ``has_lower_bound_failure` and `lower_bound_failure``

The latter four describe the bounds of the **cost** (that is, higher
is worse) of executing a BT node (or entire BT, see below). Not all
bounds might be available, and the cost for a successful execution
might be different from that for a failed one.

Think of a dangerous task where the robot might get destroyed - the
upper bound for the cost of a failure might be essentially infinite.
Conversely, for a long traversal, failure might actually have a
smaller lower bound than success, since failing at the start is not
very "expensive" to the robot.

Why?
----

Utility Functions are used when *shoving* a BT. In that case,
all available robots calculate their local Utility Function values for
the entire BT, and the "best" one gets to execute it.

What constitutes the best value and how shoving works in general is
described in :ref:`shoving`.

How?
----

To actually get Utility Function values, and to aggregate them to get
one value for an entire BT, as many node classes as possible should
implement the optional
:meth:`ros_bt_py.node.Node._do_calculate_utility` method.

That method can use all the information available to it from its
environment.
This includes Option values, ROS Topic subscriptions and Service calls.
