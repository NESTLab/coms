import unittest
import pathlib
import os
import time
import rospy
from typing import List
from std_msgs.msg import String
from coms.msg import nearby
from subprocess import call, check_output
from concurrent.futures import ThreadPoolExecutor
from coms.sim import Sim, is_sim_network_running, launch_sim_network, terminate_sim_network
from coms.constants import STATIC_LISTENER_PORT, PUB_TOPIC, SUB_TOPIC
from roslaunch import parent
from coms.utils import get_port_from, start_roscore, stop_roscore
from unittest.mock import MagicMock, Mock, patch

TEST_DIR = pathlib.Path(__file__).parent.absolute()
COMS_DIR = TEST_DIR.parent.absolute()
PROJECT_BASE_DIR = COMS_DIR.parent.absolute()
DEFAULT_NET_SIM_LAUNCH_FILE = os.path.join(PROJECT_BASE_DIR, "ros-net-sim/example/launch/testing.launch")
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
        self.assertEqual(s.LISTEN_ADDRESS, (LOOPBACK_ADDRESS, get_port_from(LOOPBACK_ADDRESS, True)))
        self.assertEqual(s.LOCAL_IPS, LAUNCH_CONFIG_LOCAL_IPS)
        self.assertEqual(s.NET_SIM_LAUNCH_FILE, DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertEqual(s.NET_PROC, None)
        self.assertEqual(s.keep_runing.locked(), False)

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

    def test_get_reachable_ips(self: unittest.TestCase) -> None:
        s1 = Sim(LAUNCH_CONFIG_LOCAL_IPS[0], DEFAULT_NET_SIM_LAUNCH_FILE)
        s2 = Sim(LAUNCH_CONFIG_LOCAL_IPS[1], DEFAULT_NET_SIM_LAUNCH_FILE)
        s1.keep_runing.acquire()
        s2.keep_runing.acquire()

        test_sim = Sim(LAUNCH_CONFIG_LOCAL_IPS[2], DEFAULT_NET_SIM_LAUNCH_FILE)
        launch = launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, LAUNCH_CONFIG_LOCAL_IPS)
        executor = ThreadPoolExecutor(max_workers=2)

        executor.submit(s1._listener)
        reachable = test_sim.get_reachable_ips('tun1')
        second_reachable = test_sim.get_reachable_ips('tun1')
        executor.submit(s2._listener)
        third_reachable = test_sim.get_reachable_ips('tun1')

        s1.keep_runing.release()
        s2.keep_runing.release()
        executor.shutdown(wait=True)
        terminate_sim_network(launch, LAUNCH_CONFIG_LOCAL_IPS)

        self.assertEqual(
            reachable,
            [(LAUNCH_CONFIG_LOCAL_IPS[0], get_port_from(LAUNCH_CONFIG_LOCAL_IPS[0], True))])

        self.assertEqual(
            second_reachable,
            [(LAUNCH_CONFIG_LOCAL_IPS[0], get_port_from(LAUNCH_CONFIG_LOCAL_IPS[0], True))])

        self.assertEqual(
            third_reachable,
            [(LAUNCH_CONFIG_LOCAL_IPS[0], get_port_from(LAUNCH_CONFIG_LOCAL_IPS[0], True)),
             (LAUNCH_CONFIG_LOCAL_IPS[1], get_port_from(LAUNCH_CONFIG_LOCAL_IPS[1], True))])

    def test_start_and_stop(self: unittest.TestCase) -> None:
        s1 = Sim(LAUNCH_CONFIG_LOCAL_IPS[0], DEFAULT_NET_SIM_LAUNCH_FILE)
        s2 = Sim(LAUNCH_CONFIG_LOCAL_IPS[1], DEFAULT_NET_SIM_LAUNCH_FILE)
        s1.start()
        for task in s1.thread_tasks:
            self.assertEqual(task.running(), True)
        s2.start()
        for task in s2.thread_tasks:
            self.assertEqual(task.running(), True)
        time.sleep(3)
        s2.stop()
        for task in s2.thread_tasks:
            self.assertEqual(task.running(), False)
        s1.stop()
        for task in s1.thread_tasks:
            self.assertEqual(task.running(), False)

    def test_listener(self: unittest.TestCase) -> None:
        with patch("coms.sim.server") as m_server:
            s1 = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
            s1._listener()
            self.assertEqual(m_server.call_count, 1)

    def test_broadcaster(self: unittest.TestCase) -> None:
        # No IP address for use
        s1 = Sim('', DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertRaises(Exception, s1._broadcaster)
        # No NIC for IP address
        s1 = Sim('68.0.206.67', DEFAULT_NET_SIM_LAUNCH_FILE)
        self.assertRaises(Exception, s1._broadcaster)
        # Clear execution with unaquired lock
        s1 = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        s1._broadcaster()
        # !IMPORTANT: If our tests block, this will be the culpret

        # Send messages to neighbors
        with patch("coms.sim.send_messsage") as s:

            s1 = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
            self.num_iterations = 1
            s1.keep_runing = MagicMock()
            s1.publish_nearby_robots = Mock()

            def locked() -> bool:
                self.num_iterations -= 1
                return self.num_iterations >= 0

            s1.keep_runing.locked = locked
            reachable = [
                (LAUNCH_CONFIG_LOCAL_IPS[0], STATIC_LISTENER_PORT),
                (LAUNCH_CONFIG_LOCAL_IPS[1], STATIC_LISTENER_PORT),
                (LAUNCH_CONFIG_LOCAL_IPS[2], STATIC_LISTENER_PORT),
                (LAUNCH_CONFIG_LOCAL_IPS[2], STATIC_LISTENER_PORT)]
            s1.get_reachable_ips = Mock(return_value=reachable)
            s1._broadcaster()
            self.assertEqual(s.call_count, len(reachable))
            self.assertEqual(s1.publish_nearby_robots.call_count, 1)

        # Send one message to one neighbor 4 times
        with patch("coms.sim.send_messsage") as s:

            s1 = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
            iterations = 4
            self.num_iterations = iterations
            s1.keep_runing = MagicMock()
            s1.publish_nearby_robots = Mock()

            def locked() -> bool:
                self.num_iterations -= 1
                return self.num_iterations >= 0

            s1.keep_runing.locked = locked
            reachable = [
                (LAUNCH_CONFIG_LOCAL_IPS[0], STATIC_LISTENER_PORT),
                (LAUNCH_CONFIG_LOCAL_IPS[1], STATIC_LISTENER_PORT)]
            s1.get_reachable_ips = Mock(return_value=reachable)
            s1._broadcaster()
            self.assertEqual(s.call_count, iterations * len(reachable))
            self.assertEqual(s1.publish_nearby_robots.call_count, iterations)

    def test_register_ros_topics(self: unittest.TestCase) -> None:
        p = start_roscore()
        rospy.init_node("some_node")

        mocked_handler = Mock()
        sim = Sim(LAUNCH_CONFIG_LOCAL_IPS[0], DEFAULT_NET_SIM_LAUNCH_FILE)
        sim.listen_handler = mocked_handler
        sim.register_ros_topics()
        self.assertNotEqual(sim.sub, None)
        self.assertNotEqual(sim.pub, None)
        pub = rospy.Publisher(name=SUB_TOPIC, data_class=String, queue_size=10)
        # NOTE: Publisher topics are created in a new thread. Allow this thread to start
        time.sleep(1)
        pub.publish(String('foo'))
        pub.publish(String('bar'))
        pub.publish(String('baz'))
        topics = rospy.get_published_topics("/")
        sim.unregister_ros_topics()
        stop_roscore(p)
        # Enusre topic was present

        pub_topic = [PUB_TOPIC, 'coms/nearby']
        print(topics)
        self.assertEqual(pub_topic in topics, True, "Sim not publishing ROS topic")
        self.assertEqual(mocked_handler.call_count, 3)

    def test_unregister_ros_topics(self: unittest.TestCase) -> None:
        p = start_roscore()
        rospy.init_node("some_node")
        sim = Sim(LAUNCH_CONFIG_LOCAL_IPS[0], DEFAULT_NET_SIM_LAUNCH_FILE)
        sim.sub = rospy.Subscriber('something', String, Mock())
        sim.pub = rospy.Publisher('pub', String, queue_size=10)
        sim.unregister_ros_topics()
        stop_roscore(p)
        self.assertEqual(sim.pub.resolved_name, None, "Publisher was not unregistered")
        self.assertEqual(sim.sub.resolved_name, None, "Subscriber was not unregistered")

    def test_publish_nearby_robots(self: unittest.TestCase) -> None:
        # No nearby robots found
        sim = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        addresses_near = []
        sim.pub = MagicMock()
        sim.pub.publish = Mock()
        sim.publish_nearby_robots(addresses_near)
        self.assertEqual(sim.pub.publish.call_count, 0, "No nearby message should be published")
        # Multiple robots nearby
        sim = Sim(LOOPBACK_ADDRESS, DEFAULT_NET_SIM_LAUNCH_FILE)
        addresses_near = [
            (LAUNCH_CONFIG_LOCAL_IPS[0], 123),
            (LAUNCH_CONFIG_LOCAL_IPS[1], 456),
            (LAUNCH_CONFIG_LOCAL_IPS[2], 789)
        ]
        sim.pub = MagicMock()
        sim.pub.publish = Mock()
        expects = nearby()
        expects.remote_addresses = [
            LAUNCH_CONFIG_LOCAL_IPS[0] + ":123",
            LAUNCH_CONFIG_LOCAL_IPS[1] + ":456",
            LAUNCH_CONFIG_LOCAL_IPS[2] + ":789"
        ]
        expects.local_address = LOOPBACK_ADDRESS + ":" + str(STATIC_LISTENER_PORT)
        sim.publish_nearby_robots(addresses_near)
        self.assertEqual(sim.pub.publish.call_count, 1)
        sim.pub.publish.assert_called_with(expects)

    # TODO: @tylerferrara
    def test_listen_handler(self: unittest.TestCase) -> None:
        pass


if __name__ == '__main__':
    unittest.main()
