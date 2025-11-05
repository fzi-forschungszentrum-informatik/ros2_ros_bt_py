# Copyright 2023 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import json
import os
import yaml
from importlib import metadata
import ament_index_python
from ament_index_python import PackageNotFoundError

from typing import Any, Optional, List

import rclpy
import rclpy.publisher
import rclpy.logging

import rosidl_runtime_py
import rosidl_runtime_py.utilities

from ros_bt_py_interfaces.msg import MessageTypes, Package, Packages
from ros_bt_py_interfaces.srv import (
    GetMessageFields,
    GetMessageConstantFields,
    SaveTree,
    GetPackageStructure,
    GetFolderStructure,
    GetStorageFolders,
)

from ros_bt_py.node import increment_name
from ros_bt_py.helpers import json_encode, build_message_field_dicts
from ros_bt_py.ros_helpers import get_message_constant_fields


def make_filepath_unique(filepath):
    name, extension = os.path.splitext(filepath)
    while os.path.exists(name + extension):
        name = increment_name(name)
    return name + extension


class PackageManager(object):
    """Provide functionality to interact with ROS messages and colcon packages."""

    def __init__(
        self,
        tree_storage_directory_paths: List[str],
        publish_message_list_callback: Optional[rclpy.publisher.Publisher] = None,
        publish_packages_list_callback: Optional[rclpy.publisher.Publisher] = None,
    ):
        self.item_id = 0
        self.tree_storage_directory_paths = []
        for path in tree_storage_directory_paths:
            self.tree_storage_directory_paths.append(
                os.path.normpath(os.path.expandvars(os.path.expanduser(path)))
            )

        self.message_list_pub = publish_message_list_callback
        self.packages_list_pub = publish_packages_list_callback

    def save_tree_to_path(
        self, request: SaveTree.Request, response: SaveTree.Response
    ) -> SaveTree.Response:
        """
        Save a tree message in the given package.

        :param ros_bt_py_msgs.srv.SaveTree request:

        If `request.filename` contains forward slashes, treat it as a relative path.
        If `request.allow_overwrite` is True, the file is overwritten, otherwise service call fails
        If `request.allow_rename` is True files will no be overwritten,
            the new file will always be renamed.

        :param ros_bt_py_msgs.srv.SaveTree response:

        :returns: :class:`ros_bt_py_msgs.src.SaveTreeResponse` or `None`

        Always returns the path under which the tree was saved
        in response.file_path in the package:// style
        """
        if request.storage_path not in self.tree_storage_directory_paths:
            response.success = False
            response.error_message = "Storage container does not exist on host!"
            return response

        # if not os.path.exists(save_filepath):
        #     response.success = False
        #     response.error_message = f"File path does not exist: {save_filepath}"
        #     return response

        try:
            save_filepath = os.path.abspath(
                os.path.join(request.storage_path, request.filepath)
            )

            split_save_filepath = save_filepath.rstrip(os.sep)  # split trailing /
            path, filename = os.path.split(split_save_filepath)

            try:
                os.makedirs(path)
            except OSError:
                if not os.path.isdir(path):
                    response.success = False
                    response.error_message = "Could not create path!"
                    response.file_path = path
                    return response

            if os.path.isdir(split_save_filepath):
                response.success = False
                response.error_message = "File path already exists as directory"
                response.file_path = path
                return response

            if os.path.isfile(split_save_filepath):
                if request.allow_rename:
                    unique_save_filepath = make_filepath_unique(split_save_filepath)
                    if os.path.isfile(unique_save_filepath):
                        response.success = False
                        response.error_message = "Rename failed"
                        response.file_path = unique_save_filepath
                        return response
                    split_save_filepath = unique_save_filepath
                else:
                    if not request.allow_overwrite:
                        response.success = False
                        response.error_message = "Overwrite not allowed"
                        response.file_path = split_save_filepath
                        return response

            # Set path to blank, this value should not be persisted
            request.tree.path = ""

            with open(split_save_filepath, "w") as save_file:
                save_file.write(f"version: {metadata.version('ros_bt_py')}\n")
                msg = rosidl_runtime_py.message_to_yaml(request.tree)
                save_file.write(msg)
            response.file_path = split_save_filepath
            response.success = True
            return response

        except IOError:
            response.success = False
            response.error_message = f'IOError on file: "{request.filepath}"'

        return response

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

        message_types = MessageTypes()
        # These reassignments makes the typing happy,
        #   because they ensure that `.append` exists
        message_types.topics = []
        message_types.services = []
        message_types.actions = []

        packages = list(rosidl_runtime_py.get_interface_packages().keys())
        for package, package_messages in rosidl_runtime_py.get_message_interfaces(
            packages
        ).items():
            for message in package_messages:
                message_types.topics.append(package + "/" + message)
        for package, package_services in rosidl_runtime_py.get_service_interfaces(
            packages
        ).items():
            for service in package_services:
                message_types.services.append(package + "/" + service)
        for package, package_actions in rosidl_runtime_py.get_action_interfaces(
            packages
        ).items():
            for action in package_actions:
                message_types.actions.append(package + "/" + action)

        self.message_list_pub.publish(message_types)

    def get_message_fields(
        self, request: GetMessageFields.Request, response: GetMessageFields.Response
    ):
        """Return the fields and field types of the provided message type."""
        try:
            message_class = rosidl_runtime_py.utilities.get_message(
                request.message_type
            )

            field_values, field_types = build_message_field_dicts(message_class())

            # Ros interfaces sometimes introduce numpy types or bytes.
            # Try their standard normalization methods.
            # If those fail, just cast to string
            def coerce_numpy_types(obj: Any):
                try:
                    return obj.tolist()
                except AttributeError:
                    rclpy.logging.get_logger("package_manager").warn(
                        f"Object of type {obj.__class__.__name__} can't be serialized properly"
                    )
                    return str(obj)

            response.fields = json.dumps(field_values, default=coerce_numpy_types)
            response.field_types = json.dumps(field_types, default=coerce_numpy_types)

            response.success = True
        except Exception as e:
            response.success = False
            response.error_message = (
                f"Could not get message fields for {request.message_type}: {e}"
            )
        return response

    # TODO Maybe instead of introducing a new message type,
    # constant_fields should also be returned as a dict?
    def get_message_constant_fields_handler(
        self,
        request: GetMessageConstantFields.Request,
        response: GetMessageConstantFields.Response,
    ):
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
        # This reassignment makes the typing happy,
        #   because it ensures that `.append` exists
        list_of_packages.packages = []

        for package, prefix in ament_index_python.get_packages_with_prefixes().items():
            self.package_paths.append(prefix)
            if not prefix.startswith("/opt/ros"):
                package_msg = Package()
                package_msg.package = package
                package_msg.path = prefix
                list_of_packages.packages.append(package_msg)

        self.packages_list_pub.publish(list_of_packages)

    def get_id(self):
        self.item_id += 1
        return self.item_id

    def reset_id(self):
        self.item_id = 0

    def path_to_dict(self, path, show_hidden=False, parent=0):
        """Turn a path into a dictionary."""
        d = {"name": os.path.basename(path), "item_id": self.get_id(), "parent": parent}
        if os.path.isdir(path):
            try:
                d["type"] = "directory"
                d["children"] = [
                    self.path_to_dict(
                        os.path.join(path, f),
                        show_hidden=show_hidden,
                        parent=d["item_id"],
                    )
                    for f in os.listdir(path)
                    if (show_hidden or not f.startswith("."))
                ]
            except OSError:
                d["type"] = "readonly_directory"
                d["children"] = []
        else:
            d["type"] = "file"
        return d

    def get_installed_package_structure(
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

    def get_folder_structure(
        self,
        request: GetFolderStructure.Request,
        response: GetFolderStructure.Response,
    ) -> GetFolderStructure.Response:
        """
        Return a listing of files and subdirectories of a folder as a jsonpickled string.

        Hides hidden files by default, unless show_hidden is set to true.
        """
        if request.storage_folder not in self.tree_storage_directory_paths:
            response.success = False
            response.error_message = (
                f'Folder "{request.storage_folder}" is not a valid storage folder:'
                f"{self.tree_storage_directory_paths}!"
            )
            return response
        self.reset_id()
        package_structure = self.path_to_dict(
            path=os.path.abspath(request.storage_folder),
            show_hidden=request.show_hidden,
        )

        response.success = True
        response.storage_folder_structure = json_encode(package_structure)

        return response

    def get_storage_folders(
        self, request: GetStorageFolders.Request, response: GetStorageFolders.Response
    ) -> GetStorageFolders.Response:
        response.storage_folders = self.tree_storage_directory_paths
        return response
