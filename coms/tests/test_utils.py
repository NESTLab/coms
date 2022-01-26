import unittest
import socket
from subprocess import check_output
from coms.utils import readable, writable, get_ip_list, get_port_from
from coms.constants import CATKIN_WS, ENCODING, NET_CONFIG


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

    def test_get_port_from(self: unittest.TestCase) -> None:
        tests = [
            ('192.168.0.1', False, 8111),
            ('192.168.0.1', True, 8221),
            ('192.168.0.19', False, 8119),
            ('192.168.0.19', True, 8219),
        ]
        for test in tests:
            got = get_port_from(ip=test[0], listener=test[1])
            self.assertEqual(got, test[2])


if __name__ == '__main__':
    unittest.main()
