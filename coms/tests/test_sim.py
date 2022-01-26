from re import M
import unittest
from subprocess import call
from typing import List
import socket
from subprocess import check_output
from coms.sim import Sim, is_sim_network_running, launch_sim_network, terminate_sim_network, gen_bound_socket, get_device_numbers, remove_net_tunnel # noqa: E501
from coms.constants import BROADCASTER_PORT, LISTENER_PORT
from roslaunch import parent
import threading
import time
from unittest.mock import Mock, MagicMock, patch


def create_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", name, "mode", "tun"])


def destroy_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", name, "mode", "tun"])


DEFAULT_NET_SIM_LAUNCH_FILE = "/root/catkin_ws/src/ros-net-sim/example/launch/gazebo.launch"


class TestSim(unittest.TestCase):

    def test_init(self: unittest.TestCase) -> None:
        s = Sim("1", "2", DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(s.L_PORT, LISTENER_PORT)
        self.assertEqual(s.L_ADDRESS, "1")
        self.assertEqual(s.B_PORT, BROADCASTER_PORT)
        self.assertEqual(s.B_ADDRESS, "2")
        self.assertEqual(s.stay_alive, True)
        self.assertEqual(s.LOCAL_IPS, ["192.168.0.1", "192.168.0.2", "192.168.0.3"])
        self.assertEqual(s.NET_SIM_LAUNCH_FILE, DEFAULT_NET_SIM_LAUNCH_FILE)

    def test_is_sim_network_running(self: unittest.TestCase) -> None:
        s: Sim = Sim("", "", DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), False, "Detected network before invocation")
        launch = launch_sim_network(s.NET_SIM_LAUNCH_FILE, s.LOCAL_IPS)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), True, "Can't detect running network")
        terminate_sim_network(launch, s.LOCAL_IPS)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), False, "Network did not appear to shut down correctly")

    def test_launch_sim_network(self: unittest.TestCase) -> None:
        s: Sim = Sim("", "", DEFAULT_NET_SIM_LAUNCH_FILE)
        launch = launch_sim_network(s.NET_SIM_LAUNCH_FILE, s.LOCAL_IPS)
        self.assertEqual(
            isinstance(launch, parent.ROSLaunchParent),
            True,
            "launch_sim_network didn't return a ROSLaunchParent object")
        out = str(check_output(["sudo", "-S", "ip", "rule", "list"]))
        self.assertEqual(out.count('lookup'), 9, "Unexpected number of ip 'lookup' rules when launching sim")
        terminate_sim_network(launch, s.LOCAL_IPS)

    def test_get_device_numbers(self: unittest.TestCase) -> None:
        # If they are, they should be manually removed
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")
        # Create four tunnel devices
        create_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [0, 1, 2, 3], "Additional tunnel devices not observed")
        destroy_tunnels(["tun0", "tun1", "tun2", "tun3"])
        self.assertEqual(get_device_numbers(), [], "Detected unexpected tunnel device")

    def test_gen_bound_socket(self: unittest.TestCase) -> None:
        sock: socket.socket = gen_bound_socket(('127.0.0.1', 8831))
        self.assertEqual(sock.family, socket.AF_INET)
        self.assertEqual(sock.type, socket.SOCK_STREAM)
        sock.close()

    def test_remove_net_tunnel(self: unittest.TestCase) -> None:
        local_ips = ["192.168.0.1", "192.168.0.2", "192.168.0.3"]
        launch = launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, local_ips)
        check_output(["sudo", "ip", "rule", "list"])
        terminate_sim_network(launch, local_ips)

    def test_start_and_stop(self: unittest.TestCase) -> None:
        s = Sim("192.168.0.1", "192.168.0.1", DEFAULT_NET_SIM_LAUNCH_FILE)
        s.start()
        for task in s.thread_tasks:
            self.assertEqual(task.running(), True, "Not all tasks are running after invoking Sim.start()")
        s.stop()
        for task in s.thread_tasks:
            self.assertEqual(task.done(), True, "Not all tasks stopped after invoking Sim.stop()")

    # def test_start_listener(self: unittest.TestCase) -> None:
    #     # ====== Listener never accepts client messages ======
    #     # Startup network
    #     local_ips = ["192.168.0.1", "192.168.0.2", "192.168.0.3"]
    #     s = Sim("192.168.0.1", "192.168.0.1", DEFAULT_NET_SIM_LAUNCH_FILE)
    #     launch = launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, local_ips)
    #     # Mock kill_thread_event
    #     fake_event = MagicMock()
    #     fake_event.is_set = Mock(return_value=True)
    #     fake_client_sock = Mock()
    #     fake_sock = MagicMock()
    #     fake_sock.listen = Mock()
    #     fake_sock.accept = (fake_client_sock, "some_addr")
    #     with patch('coms.sim.gen_bound_socket', return_value=fake_sock) as mock_gen_bound_socket:
    #         s.kill_thread_event = fake_event
    #         listener: threading.Thread = threading.Thread(target=s.start_listener)
    #         listener.start()
    #         listener.join()

    #     mock_gen_bound_socket.assert_called_once()
    #     self.assertEqual(fake_event.is_set.call_count, 1, "Sim.kill_thread_event.is_set() was never called")
    #     self.assertEqual(fake_sock.listen.call_count, 1, "Listener socket was never given call to listen()")
    #     self.assertEqual(fake_client_sock.call_count, 0, "Client socket should not have been called")
    #     terminate_sim_network(launch, local_ips)

        # ====== Listener accepts one client message ======
        # TODO:


if __name__ == '__main__':
    unittest.main()
