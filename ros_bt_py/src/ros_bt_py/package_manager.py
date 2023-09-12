# Copyright 2023 FZI Forschungszentrum Informatik

import os
from ament_index_python import PackageNotFoundError
from rclpy.utilities import ament_index_python

from typing import Optional

import rclpy
import rclpy.publisher
import rclpy.logging

import rosidl_runtime_py
import rosidl_runtime_py.utilities

from ros_bt_py_interfaces.msg import Message, Messages, Package, Packages
from ros_bt_py_interfaces.srv import (
    GetMessageFields,
    SaveTree,
    GetPackageStructure,
    FixYaml,
)

from ros_bt_py.node import increment_name
from ros_bt_py.helpers import (
    fix_yaml,
    remove_input_output_values,
    json_encode,
)
from ros_bt_py.ros_helpers import get_message_constant_fields


class PackageManager(object):
    """Provide functionality to interact with ROS messages and colcon packages."""

    def __init__(
        self,
        publish_message_list_callback: Optional[rclpy.publisher.Publisher] = None,
        publish_packages_list_callback: Optional[rclpy.publisher.Publisher] = None,
    ):
        self.item_id = 0

        self.message_list_pub = publish_message_list_callback
        self.packages_list_pub = publish_packages_list_callback

    def publish_message_list(self):
        """
        Publish a list of all ROS messages/services available on the system.

        Uses a similar strategy to rosmsg/rossrv to detect message/service files.
        """
        if self.message_list_pub is None:
            rclpy.logging.get_logger("package_manager").warn(
                "No callback for publishing message list data provided."
            )
            return

        messages = []

        packages = list(ament_index_python.get_packages_with_prefixes().keys())
        for package, package_messages in rosidl_runtime_py.get_message_interfaces(
            packages
        ).items():
            for message in package_messages:
                messages.append(
                    Message(msg=package + "/" + message, service=False, action=False)
                )
        for package, package_services in rosidl_runtime_py.get_service_interfaces(
            packages
        ).items():
            for service in package_services:
                messages.append(
                    Message(msg=package + "/" + service, service=True, action=False)
                )
        for package, package_actions in rosidl_runtime_py.get_action_interfaces(
            packages
        ).items():
            for action in package_actions:
                messages.append(
                    Message(msg=package + "/" + action, service=False, action=True)
                )

        msg = Messages()
        msg.messages = messages
        self.message_list_pub.publish(msg)

    def get_message_fields(
        self, request: GetMessageFields.Request, response: GetMessageFields.Response
    ):
        """Return the jsonpickled fields of the provided message type."""
        try:
            message_class = None
            if request.service:
                message_class = rosidl_runtime_py.utilities.get_service(
                    request.message_type
                )
            else:
                if request.action:
                    message_class = rosidl_runtime_py.utilities.get_action(
                        request.message_type
                    )
                else:
                    message_class = rosidl_runtime_py.utilities.get_message(
                        request.message_type
                    )
            for field in message_class._fields_and_field_types:
                response.field_names.append(field.strip())
            response.fields = json_encode(
                rosidl_runtime_py.message_to_ordereddict(message_class())
            )
            response.success = True
        except Exception as e:
            response.success = False
            response.error_message = (
                f"Could not get message fields for {request.message_type}: {e}"
            )
        return response

    def get_message_constant_fields_handler(
        self, request: GetMessageFields.Request, response: GetMessageFields.Response
    ):
        if request.service or request.action:
            # not supported yet
            response.success = False
            response.error_message = (
                "Constant message fields for services are not yet supported"
            )
        else:
            try:
                message_class = rosidl_runtime_py.utilities.get_message(
                    request.message_type
                )
                response.field_names = get_message_constant_fields(message_class)
                response.success = True
            except Exception as e:
                response.success = False
                response.error_message = (
                    f"Could not get message fields for {request.message_type}: {e}"
                )
        return response

    def publish_packages_list(self):
        if self.packages_list_pub is None:
            rclpy.logging.get_logger("package_manager").warn(
                "No callback for publishing packages list data provided."
            )
            return
        self.package_paths = []
        list_of_packages = Packages()
        for package, prefix in ament_index_python.get_packages_with_prefixes().items():
            self.package_paths.append(prefix)
            if not prefix.startswith("/opt/ros"):
                package_msg = Package()
                package_msg.package = package
                package_msg.path = prefix
                list_of_packages.packages.append(package_msg)
                break

        self.packages_list_pub.publish(list_of_packages)

    def get_id(self):
        self.item_id += 1
        return self.item_id

    def reset_id(self):
        self.item_id = 0

    def path_to_dict(self, path, show_hidden=False, parent=0):
        """Turn a path into a dictionary."""
        d = {"name": os.path.basename(path)}
        d["item_id"] = self.get_id()
        d["parent"] = parent
        if os.path.isdir(path):
            d["type"] = "directory"
            d["children"] = [
                self.path_to_dict(
                    os.path.join(path, f), show_hidden=show_hidden, parent=d["item_id"]
                )
                for f in os.listdir(path)
                if (show_hidden or not f.startswith("."))
            ]
        else:
            d["type"] = "file"
        return d

    def get_package_structure(
        self,
        request: GetPackageStructure.Request,
        response: GetPackageStructure.Response,
    ) -> GetPackageStructure.Response:
        """
        Return a listing of files and subdirectories of a ROS package as a jsonpickled string.

        Hides hidden files by default, unless show_hidden is set to true.
        """
        try:
            package_path = ament_index_python.get_package_share_directory(
                request.package
            )
            self.reset_id()
            package_structure = self.path_to_dict(
                path=package_path, show_hidden=request.show_hidden
            )

            response.success = True
            response.package_structure = json_encode(package_structure)
        except PackageNotFoundError:
            response.success = False
            response.error_message = f'Package "{request.package}" does not exist'

        return response

    def make_filepath_unique(self, filepath):
        name, extension = os.path.splitext(filepath)
        while os.path.exists(name + extension):
            name = increment_name(name)
        return name + extension

    def save_tree(
        self, request: SaveTree.Request, response: SaveTree.Response
    ) -> SaveTree.Response:
        """
        Save a tree message in the given package.

        :param ros_bt_py_msgs.srv.SaveTree request:

        If `request.filename` contains forward slashes, treat it as a relative path.
        If `request.allow_overwrite` is True, the file is overwritten, otherwise service call fails
        If `request.allow_rename` is True files will no be overwritten,
            the new file will always be renamed.

        :returns: :class:`ros_bt_py_msgs.src.SaveTreeResponse` or `None`

        Always returns the path under which the tree was saved
        in response.file_path in the package:// style
        """
        # remove input and output values from nodes
        request.tree = remove_input_output_values(tree=request.tree)
        if not os.path.exists(request.filepath):
            response.success = False
            response.error_message = "File path does not exist!"
            return response
        try:
            save_path = os.path.join(request.filepath, request.filename)

            save_path = save_path.rstrip(os.sep)  # split trailing /
            path, filename = os.path.split(save_path)

            # set tree name to filename
            request.tree.name = filename

            try:
                os.makedirs(path)
            except OSError:
                if not os.path.isdir(path):
                    response.success = False
                    response.error_message = "Could not create path"
                    return response

            if os.path.isdir(save_path):
                response.success = False
                response.error_message = "File path already exists as directory"
                return response

            if os.path.isfile(save_path):
                if request.allow_rename:
                    save_path = self.make_filepath_unique(save_path)
                    if os.path.isfile(save_path):
                        response.success = False
                        response.error_message = "Rename failed"
                        return response
                else:
                    if not request.allow_overwrite:
                        response.success = False
                        response.error_message = "Overwrite not allowed"
                        return response

            with open(save_path, "w") as save_file:
                msg = rosidl_runtime_py.message_to_yaml(request.tree)
                fix_yaml_response = fix_yaml(request=FixYaml.Request(broken_yaml=msg))
                save_file.write(fix_yaml_response.fixed_yaml)
            response.success = True
            return response

        except IOError:
            response.success = False
            response.error_message = f'IOError on file: "{request.filename}"'

        return response
