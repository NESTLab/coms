import unittest
import socket
import rospy
from typing import List
from subprocess import check_output, call
from coms.utils import readable, writable, get_ip_list, get_interface_from_ip, get_device_numbers, gen_bound_socket, start_roscore, stop_roscore, addr_to_str # noqa: E501
from coms.constants import CATKIN_WS, ENCODING, NET_CONFIG
from roslaunch.parent import ROSLaunchParent


def create_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", name, "mode", "tun"])


def destroy_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", name, "mode", "tun"])


class TestUtils(unittest.TestCase):

    def test_readable(self: unittest.TestCase) -> None:
        sock = socket.socket()
        self.assertEqual(readable(sock), True, msg="Socket not found to be readable")
        sock.close()
        self.assertEqual(readable(sock), False, msg="Closed socket found to be readable")

    def test_writable(self: unittest.TestCase) -> None:
        sock = socket.socket()
        self.assertEqual(writable(sock), True, msg="Socket not found to be writable")
        sock.close()
        self.assertEqual(writable(sock), False, msg="Closed socket found to be writable")

    def test_get_ip_list(self: unittest.TestCase) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, NET_CONFIG), shell=True)
        local_ips = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.assertEqual(local_ips, ["192.168.0.1", "192.168.0.2", "192.168.0.3"])

    def test_get_interface_from_ip(self: unittest.TestCase) -> None:
        # Loopback device will always have the same IP address
        self.assertEqual(
            get_interface_from_ip('127.0.0.1'),
            'lo',
            "Unexpected network interface found for loopback device")

    def test_get_device_numbers(self: unittest.TestCase) -> None:
        # If they are, they should be manually removed
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")
        # Create four tunnel devices
        create_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [0, 1, 2, 3], "Additional tunnel devices not observed")
        destroy_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")

    def test_gen_bound_socket(self: unittest.TestCase) -> None:
        sock = gen_bound_socket('127.0.0.1')
        self.assertEqual(sock.family, socket.AF_INET, "Unexpected socket family")
        self.assertEqual(sock.type, socket.SOCK_STREAM, "Unexpected socket type")
        sock.close()
        self.assertRaises(Exception, gen_bound_socket, '')

    def test_start_roscore(self: unittest.TestCase) -> None:
        parent: ROSLaunchParent = start_roscore()
        self.assertEqual(parent.is_core, True, "is_core option NOT set in ROSLaunchParent")
        self.assertEqual(rospy.get_published_topics("/"), [], "roscore not running")
        parent.shutdown()

    def test_stop_roscore(self: unittest.TestCase) -> None:
        parent: ROSLaunchParent = start_roscore()
        self.assertEqual(rospy.get_published_topics("/"), [], "roscore not running")
        stop_roscore(parent)
        self.assertRaises(OSError, rospy.get_published_topics, "/")

    def test_addr_to_str(self: unittest.TestCase) -> None:
        tests = [
            (("", 0), ":0"),
            (("192.168.0.1", 177), "192.168.0.1:177"),
            (("1", 1), "1:1")
        ]
        for t in tests:
            got = addr_to_str(t[0])
            expects = t[1]
            self.assertEqual(got, expects)


if __name__ == '__main__':
    unittest.main()
