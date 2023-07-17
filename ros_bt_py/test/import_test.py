import importlib
import pkgutil
import os
import unittest

import ros_bt_py


class ImportTest(unittest.TestCase):
    """Check whether all of our modules import fine."""

    def test_imports(self):
        for loader, name, is_pkg in pkgutil.walk_packages(ros_bt_py.__path__):
            full_name = ros_bt_py.__name__ + "." + name
            importlib.import_module(full_name)


if __name__ == "__main__":
    unittest.main()
