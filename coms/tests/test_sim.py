import unittest
from typing import List
from subprocess import call, check_output
from concurrent.futures import ThreadPoolExecutor
from coms.sim import Sim, is_sim_network_running, launch_sim_network, terminate_sim_network
from coms.constants import STATIC_LISTENER_PORT
from roslaunch import parent


DEFAULT_NET_SIM_LAUNCH_FILE = "/root/catkin_ws/src/ros-net-sim/example/launch/gazebo.launch"
LAUNCH_CONFIG_LOCAL_IPS = ["192.168.0.1", "192.168.0.2", "192.168.0.3"]
LOOPBACK_ADDRESS = '127.0.0.1'


def create_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "add", "dev", name, "mode", "tun"])


def destroy_tunnels(device_names: List) -> None:
    for name in device_names:
        call(["sudo", "-S", "ip", "tuntap", "del", "dev", name, "mode", "tun"])


class TestSim(unittest.TestCase):

    def test_init(self: unittest.TestCase) -> None:
        s = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(s.LISTEN_ADDRESS, (LOOPBACK_ADDRESS, STATIC_LISTENER_PORT))
        self.assertEqual(s.LOCAL_IPS, LAUNCH_CONFIG_LOCAL_IPS)
        self.assertEqual(s.NET_SIM_LAUNCH_FILE, DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(type(s.thread_executor), ThreadPoolExecutor)
        self.assertEqual(s.NET_PROC, None)
        self.assertEqual(s.NIC, '')

    def test_is_sim_network_running(self: unittest.TestCase) -> None:
        s: Sim = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), False, "Detected network before invocation")
        launch = launch_sim_network(s.NET_SIM_LAUNCH_FILE, s.LOCAL_IPS)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), True, "Can't detect running network")
        terminate_sim_network(launch, s.LOCAL_IPS)
        self.assertEqual(is_sim_network_running(s.LOCAL_IPS), False, "Network did not appear to shut down correctly")

    def test_launch_sim_network(self: unittest.TestCase) -> None:
        s: Sim = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        out = str(check_output(["sudo", "-S", "ip", "rule", "list"]))
        self.assertEqual(out.count('lookup'), 3, "Unexpected number of ip 'lookup' rules when before launching sim")
        launch = launch_sim_network(s.NET_SIM_LAUNCH_FILE, s.LOCAL_IPS)
        self.assertEqual(
            isinstance(launch, parent.ROSLaunchParent),
            True,
            "launch_sim_network didn't return a ROSLaunchParent object")
        out = str(check_output(["sudo", "-S", "ip", "rule", "list"]))
        self.assertEqual(out.count('lookup'), 9, "Unexpected number of ip 'lookup' rules when launching sim")
        terminate_sim_network(launch, s.LOCAL_IPS)

    def test_remove_net_tunnel(self: unittest.TestCase) -> None:
        launch = launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, LAUNCH_CONFIG_LOCAL_IPS)
        out = str(check_output(["sudo", "ip", "rule", "list"]))
        self.assertEqual(out.count('192.168.0'), 3)
        terminate_sim_network(launch, LAUNCH_CONFIG_LOCAL_IPS)

    def test_start_and_stop(self: unittest.TestCase) -> None:
        s = Sim(LAUNCH_CONFIG_LOCAL_IPS[0], DEFAULT_NET_SIM_LAUNCH_FILE)
        s.start()
        for task in s.thread_tasks:
            self.assertEqual(task.running(), True)
        s.stop()
        for task in s.thread_tasks:
            self.assertEqual(task.running(), False)
            self.assertEqual(task.done(), True)


if __name__ == '__main__':
    unittest.main()
