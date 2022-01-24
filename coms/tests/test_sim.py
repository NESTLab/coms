import unittest
from subprocess import call
from typing import List
import socket
from coms.sim import Sim, gen_bound_socket, is_sim_network_running, launch_sim_network, terminate_sim_network, get_device_numbers # noqa: E501
from coms.constants import BROADCASTER_PORT, LISTENER_PORT


def create_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", name, "mode", "tun"])


def destroy_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", name, "mode", "tun"])


class TestSim(unittest.TestCase):

    def test_is_sim_network_running(self: unittest.TestCase) -> None:
        self.assertEqual(is_sim_network_running(), False, "Detected network before invocation")
        proc = launch_sim_network()
        self.assertEqual(is_sim_network_running(), True, "Can't detect running network")
        terminate_sim_network(proc)
        self.assertEqual(is_sim_network_running(), False, "Network did not appear to shut down correctly")

    def test_launch_sim_network(self: unittest.TestCase) -> None:
        proc = launch_sim_network()
        terminate_sim_network(proc)

    def test_get_device_numbers(self: unittest.TestCase) -> None:
        # If they are, they should be manually removed
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")
        # Create four tunnel devices
        create_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [0, 1, 2, 3], "Additional tunnel devices not observed")
        destroy_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")

    def test_init(self: unittest.TestCase) -> None:
        s = Sim("1", "2")
        self.assertEqual(s.L_PORT, LISTENER_PORT)
        self.assertEqual(s.L_ADDRESS, "1")
        self.assertEqual(s.B_PORT, BROADCASTER_PORT)
        self.assertEqual(s.B_ADDRESS, "2")
        self.assertEqual(s.stay_alive, True)
        self.assertEqual(s.LOCAL_IPS, ["192.168.0.1", "192.168.0.2", "192.168.0.3"])

    def test_start_and_stop(self: unittest.TestCase) -> None:
        s = Sim("192.168.0.1", "192.168.0.1")
        s.start()
        self.assertEqual(len(s.threads), 2)
        for t in s.threads:
            self.assertEqual(t.is_alive(), True)
        s.stop()
        for t in s.threads:
            self.assertEqual(t.is_alive(), False)

    def test_gen_bound_socket(self: unittest.TestCase) -> None:
        sock: socket.socket = gen_bound_socket('127.0.0.1', 8831)
        self.assertEqual(sock.family, socket.AF_INET)
        self.assertEqual(sock.type, socket.SOCK_STREAM)

    # def test_remove_net_tunnel(self: unittest.TestCase) -> None:
    #     proc = launch_sim_network()
    #     check_output(["sudo", "ip", "rule", "list"])
    #     for n in get_device_numbers():
    #         remove_net_tunnel(n)
    #     terminate_sim_network(proc)


if __name__ == '__main__':
    unittest.main()
