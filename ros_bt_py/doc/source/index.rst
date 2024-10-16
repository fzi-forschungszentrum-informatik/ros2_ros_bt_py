.. ros_bt_py documentation master file, created by
   sphinx-quickstart on Sun Jan 28 18:00:15 2018.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

#####################################
Welcome to ros_bt_py's documentation!
#####################################

************
Introduction
************

ros_bt_py is a Python package for writing and executing Behavior Trees.

It is built to enable high-level (aka task level) mission control for
robots, similar to BehaviorTree.Cpp_, SMACH_, FlexBE_ or RAFCON_.

The roy_bt_py repository is home to multiple elements enabling you to quickly get started with BT
development without the need to write any code:

* The basic framework holding ros_bt_by together.
* A "standard library" of Behavior Tree nodes (i.e. subclasses of :class:`ros_bt_py.node.Node`)
  that already allow the construction of complex robot mission Behavior Trees. While these will let
  you do basically whatever you want, :ref:`writing your own nodes<creating-nodes>` might be
  helpful after you are familiar with the library.
* The ros_bt_py web GUI, which should in general be your interface to interact with the library.
  Generating trees through code is currently possible, but neither supported nor documented.

.. _BehaviorTree.Cpp: https://www.behaviortree.dev/
.. _SMACH: http://wiki.ros.org/smach
.. _FlexBE: http://philserver.bplaced.net/
.. _RAFCON: https://rafcon.readthedocs.io/en/latest/

*****************
Mission Statement
*****************

ros_bt_py was created with the following goals in mind:

* Fully ROS compatible.
* Mission control for Robots, meaning long runing processes.
* Possiblity to distribute it among multiple robots to enable cooperation in robot Teams.
* Extendible, understandable and typesafe implementation.

It is meant as a high level control option similar to SMACH or FlexBE.

*****************
Table of Contents
*****************

.. toctree::
    :maxdepth: 2

    getting_started
    creating_node_classes
    testing_node_classes
    using_custom_node_classes
    utility_functions
    api


******************
Indices and tables
******************

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
