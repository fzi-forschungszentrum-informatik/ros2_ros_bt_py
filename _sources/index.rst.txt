.. ros_bt_py documentation master file, created by
   sphinx-quickstart on Sun Jan 28 18:00:15 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to ros_bt_py's documentation!
=====================================

ros_bt_py is a Python package for Behavior Trees

It is built to enable high-level (aka task level) mission control for
robots, similar to BehaviorTree.Cpp_, SMACH_, FlexBE_ or RAFCON_.

The library holds both the basic framework holding ros_bt_by together
and a sort of "standard library" of Behavior Tree nodes
(i.e. subclasses of :class:`ros_bt_py.node.Node`) that allow the
construction of complex robot mission Behavior Trees.

For information on how to create your own node classes, see :ref:`creating-nodes`

.. _BehaviorTree.Cpp: https://www.behaviortree.dev/
.. _SMACH: http://wiki.ros.org/smach
.. _FlexBE: http://philserver.bplaced.net/
.. _RAFCON: https://rafcon.readthedocs.io/en/latest/


Contents:

.. toctree::
    :maxdepth: 2

    getting_started
    creating_node_classes
    testing_node_classes
    utility_functions
    api


Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
