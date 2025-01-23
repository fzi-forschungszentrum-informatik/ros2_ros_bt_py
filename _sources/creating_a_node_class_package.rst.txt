.. _creating-packages:

#############################
Creating a Node Class package
#############################

When adding more complex node classes it is strongly advised, that you create your own Python
package for them, so you don't have to fork (or even have uncommited changes!!) in your
``ros_bt_py`` repository.

****************
Package Creation
****************

You can use the standard ament package creation methods to create an empty python package.

.. code-block:: bash

  ros2 pkg create --build-type ament_python <package_name>

Afterwards simply create your Node Classes inside the `package_name` module.
Always remember to add your modules inside the `packages` tag of your `setup.py` for them to be
recognized as modules.

***************
Example Package
***************

Below is the example sturcture of a package called ``test_behaviors`` with a ``node`` module for
custom node classes inside the ``example.py`` file.

.. code-block:: text

   test_behaviors/
   ├── test_behaviors/
   │   ├── __init__.py
   │   └── nodes/
   │       ├── __init__.py
   │       └── example.py
   ├── test/
   │   ├── test_copyright.py
   │   ├── test_flake8.py
   │   └── test_pep257.py
   ├── setup.py
   ├── setup.cfg
   └── README.md

The ``setup.py`` file in this case should look like this:

.. code-block:: python

   import os
   from glob import glob
   from setuptools import setup

   package_name = 'test_behaviors'

   setup(
        name=package_name,
        version='0.0.0',
        packages=[
            "test_behaviors",
            "test_behaviors.nodes",
        ],
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
            (os.path.join('share', package_name, 'trees'), glob(os.path.join('trees', '*.yaml')))
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='',
        maintainer_email='',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
            ],
        },
   )
