#!/usr/bin/env python
# Copyright 2023 FZI Forschungszentrum Informatik

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from netifaces import interfaces, ifaddresses, AF_INET


def ip4_addresses():
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


def main():
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

    return 0


if __name__ == "__main__":
    main()
