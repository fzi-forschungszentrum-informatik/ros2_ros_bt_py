.. _creating-nodes:

Creating Node Classes
=====================

Say you are building a Behavior Tree, and want to do something more
complex than the node classes in ``ros_bt_py.nodes`` or
``ros_bt_py.ros_nodes`` can support.

Or maybe they can, but you want to make your tree look less cluttered
by combining a bunch of steps into a single node.

To do that, all you have to do is extend the
:class:`ros_bt_py.node.Node` class! There are basically three steps to
this:

1. Create a new class and decorate it with :meth:`ros_bt_py.node.define_bt_node`
2. Fill in the :class:`ros_bt_py.node_config.NodeConfig` parameter to ``define_bt_node``
3. Implement the :code:`_do_` methods

You really also should do step 4:

4. Test your new node!


1. Create a new class
---------------------

Let's start with the skeleton of a new node class::

  import rclpy

  # Import the ROS message class under a different name
  # to avoid confusion. We need it for the node state names.
  from ros_bt_py_interfaces.msg import Node as NodeMsg

  # We need these to define a Node.
  from ros_bt_py.node import Node, define_bt_node
  from ros_bt_py.node_config import NodeConfig, OptionRef

  @define_bt_node(NodeConfig(
      options={},
      inputs={},
      outputs={},
      max_children=None))
  class MyAwesomeNode(Node):
      """This Node makes everything so much easier!"""
      def _do_setup(self):
          pass

      def _do_tick(self):
          return NodeMsg.FAILED

      def _do_shutdown(self):
          pass

      def _do_reset(self):
          return NodeMsg.IDLE

      def _do_untick(self):
          return NodeMsg.IDLE

      # Uncomment this if your node provides a utility calculation
      #
      # def _do_calculate_utility(self):
      #     pass

If you load this module into the BT editor, it will actually work - it
won't *do* anything, but you can add it to a Behavior Tree, tick it,
untick, reset and shut it down to your heart's content.

2. Fill in the NodeConfig
-------------------------

That's one step down. Let's move on to filling in the Node interface::

  @define_bt_node(NodeConfig(
      options={},
      inputs={},
      outputs={},
      max_children=None))
  class MyAwesomeNode(Node):

Let's say we want to create a Node that has at most four children, and
you decide which one to tick using an Input.

This means our :class:`ros_bt_py.node_config.NodeConfig` object should
look like this::

  @define_bt_node(NodeConfig(
      options={},
      inputs={'run_child_index': int},
      outputs={},
      max_children=4))
  class MyAwesomeNode(Node):

If we didn't want to limit the number of children, we'd leave
:code:`max_children` at :code:`None`.  To tell the decorator your node
cannot have any children, set :code:`max_children=0`.

3. Implement the :code:`_do_` methods
-------------------------------------

Finally, it's time to actually implement the :code:`_do_` methods.
Luckily, these are fairly straightforward for our example.

Most nodes that have children will want to call *their*
:meth:`setup()` methods in :meth:`_do_setup()`, like so::

  def _do_setup(self):
      for child in self.children:
          child.setup()

The same goes for :meth:`_do_shutdown()`, :meth:`_do_reset()` and
:meth:`_do_untick()`. Of course, depending on your application, there
might be important things to do in these, particularly
:meth:`_do_setup()` and :meth:`_do_shutdown()`.

But :meth:`_do_tick()` is where the magic happens in our case::

  def _do_tick(self):
      return self.children[self.inputs['run_child_index']].tick()

This reads the input :code:`run_child_index` we've defined using
:meth:`ros_bt_py.node.define_bt_node`, and uses it to index into the
list of our node's children.

With all of the :code:`_do_` methods implemented the complete code
looks like this::

  import rclpy

  # Import the ROS message class under a different name
  # to avoid confusion. We need it for the node state names.
  from ros_bt_py_interfaces.msg import Node as NodeMsg

  # We need these to define a Node.
  from ros_bt_py.node import Node, define_bt_node
  from ros_bt_py.node_config import NodeConfig, OptionRef

  @define_bt_node(NodeConfig(
      options={},
      inputs={'run_child_index': int},
      outputs={},
      max_children=4))
  class MyAwesomeNode(Node):
      """This Node makes everything so much easier!"""
      def _do_setup(self):
          for child in self.children:
              child.setup()

      def _do_tick(self):
          return self.children[self.inputs['run_child_index']].tick()

      def _do_shutdown(self):
          for child in self.children:
              child.shutdown()

      def _do_reset(self):
          for child in self.children:
              child.reset()
          return NodeMsg.IDLE

      def _do_untick(self):
          for child in self.children:
              child.untick()
          return NodeMsg.IDLE

      # Uncomment this if your node provides a utility calculation
      #
      # def _do_calculate_utility(self):
      #     pass

So we're done and ready to roll!

4. Test your node!
------------------

..\.Or are we?

Of course we're not. You should thoroughly test any node class, and if
you do test :class:`MyAwesomeNode` you should find a few things to
improve.

Check out :ref:`testing-nodes` for some advice on how to test.

Finally, in some cases it might also make sense to implement :meth:`_do_calculate_utility()`. More on that over at :ref:`utility-functions` .
