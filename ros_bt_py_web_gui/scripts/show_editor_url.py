#!/usr/bin/env python3

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
from typing import List
import rclpy
from rclpy.node import Node

from netifaces import interfaces, ifaddresses, AF_INET


def ip4_addresses() -> List[str]:
    ip_list: List[str] = []
    for interface in interfaces():
        try:
            for link in ifaddresses(interface)[AF_INET]:
                ip_list.append(link["addr"])
        except KeyError:
            continue
    return ip_list


def print_url_future(node: Node) -> None:
    hostname_param = node.get_parameter("hostname")
    port_param = node.get_parameter("port")
    hostname = str(hostname_param.value)
    port = int(port_param.value)

    hostnames = []
    if hostname == "0.0.0.0":
        hostnames = ip4_addresses()
    else:
        hostnames.append(hostname)

    suffix = f":{port}/index.html"
    hostnames_str: List[str] = []
    max_hostname_len = 30
    for hostname in hostnames:
        hostname_prefix = f"# http://{hostname}"
        hostname_len = len(hostname_prefix + suffix + " #")
        if hostname_len > max_hostname_len:
            max_hostname_len = hostname_len

        hostnames_str.append(hostname_prefix)

    padding_str = "{:#>{w}}".format("", w=max_hostname_len)
    node.get_logger().info(padding_str)
    for hostname_prefix in hostnames_str:
        hostname_padding_str = "{: >{w}}".format(
            "", w=max_hostname_len - len(hostname_prefix) - len(suffix) - 2
        )
        node.get_logger().info(hostname_prefix + suffix + f"{hostname_padding_str} #")
    node.get_logger().info(padding_str)


def main() -> None:
    rclpy.init()

    node = Node("url_printer")
    node.declare_parameters(
        namespace="",
        parameters=[
            ("hostname", rclpy.Parameter.Type.STRING),
            ("port", rclpy.Parameter.Type.INTEGER),
        ],
    )

    print_url_future(node=node)


if __name__ == "__main__":
    main()
