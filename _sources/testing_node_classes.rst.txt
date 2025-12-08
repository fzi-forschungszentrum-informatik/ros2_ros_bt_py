.. _testing-nodes:

####################
Testing Node Classes
####################

.. toctree::
   :maxdepth: 2

   Creating a Test Folder and Unit Test Script
   Running your Unit Tests
   Adding your Unit Tests to CMakeLists.txt
   Adding your Unit Tests to setup.py
   Running Tests via colcon

Even though a lot of errors are harder to make and/or easier to catch
with :mod:`ros_bt_py` than with other libraries, you should still
test any new node classes you make.

The easiest way to to this is by adding tests to your ROS2 package.  To
do that, you simply create a new folder, slap some test scripts in
there, and add a few lines to your :code:`CMakeLists.txt` or :code:`setup.py`, and
:code:`package.xml`.  I'll walk you through it:

*******************************************
Creating a Test Folder and Unit Test Script
*******************************************

The most common way to write Python unit tests in ROS2 is the
:mod:`pytest` module.

So, within your ROS package, create a new folder called
:code:`tests/`.
Next, create a new file called :code:`test_my_node.py` and put the
following code in it

.. code-block:: python
  :linenos:

  import pytest

  # Useful to have the state names to compare against
  from ros_bt_py_interfaces.msg import Node as NodeMsg

  from my_pacakge.nodes.my_awesome_node import MyAwesomeNode

  def test_setup():
    awesome = MyAwesomeNode()
    awesome.setup()

    # Any node should be IDLE after calling setup()
    assert awesome.state == NodeMsg.IDLE

A lot of the names here are important, because pytest uses the function name to find
and run tests.  Your test script must start with the word "test".

***********************
Running your Unit Tests
***********************

Now, to actually *run* your unit tests, simply go to the package folder and run

.. code-block:: bash

  pytest

****************************************
Adding your Unit Tests to CMakeLists.txt
****************************************

If you want your tests to be run by the CI pipeline (and who
doesn't?) and you use CMakeLists.txt to build your package,
you'll also have to tell the build system about them.

More info can be found in the ROS2 tutorials_ .

.. _tutorials: https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Python-Documentation.html

Don't worry, it's easy - just add the following lines to your
`CMakeLists.txt`, and nose will do the rest

.. code-block:: cmake
  :linenos:

  if (BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)

    # Add the python test files you wrote here:
    set(_pytest_tests
      tests/test_a.py
      tests/test_b.py
      # Add other test files here
    )
    foreach(_test_path ${_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  endif()

**********************************
Adding your Unit Tests to setup.py
**********************************

When using an ``ament_python`` package, pytest based tests are automatically picked up by the system.
No additional steps should be required.

************************
Running Tests via colcon
************************

Now that you've registered your tests with ROS2 and colcon, you can use colcon
to run them

.. code-block:: bash

  colcon test
