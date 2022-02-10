from __future__ import annotations
import time
import socket
import roslaunch
import rospy
from threading import Lock
from typing import List, Tuple
from subprocess import check_output, call
from std_msgs.msg import String
from coms.constants import QUICK_WAIT_TIMER, RESPONSE_TIMEOUT, BROADCAST_INTERVAL, ENCODING, CATKIN_WS, NET_CONFIG, SUB_TOPIC, PUB_TOPIC # noqa: E501
from coms.utils import get_interface_from_ip, get_port_from, get_ip_list, get_device_numbers
from coms.server import server, send_messsage
from concurrent.futures import ThreadPoolExecutor, Future


class Sim():
    LOCAL_IPS: List[str] = []                               # List of ip addresses defined in configuration file
    LISTEN_ADDRESS: Tuple[str, int] = ()                    # Address bound to the listener's TCP socket server
    NET_PROC: roslaunch.parent.ROSLaunchParent = None       # ROS specific launch object - for starting sim network
    NET_SIM_LAUNCH_FILE: str = ""                           # Path to sim network launch file
    thread_executor: ThreadPoolExecutor = None              # Executor for managing threaded operations
    thread_tasks: List[Future] = []                         # List of Future objects for obtaining of thread stats
    keep_runing: Lock = None                                # Mutex for threads to determin if they should keep running
    pub: rospy.Publisher = None                             # ROS Publisher for sending msg responses to other nodes
    sub: rospy.Subscriber = None                            # ROS Subscriber for listening to message requests

    def __init__(self: Sim, address: str, net_sim_launch_file: str) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, NET_CONFIG), shell=True)
        self.LOCAL_IPS = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.NET_SIM_LAUNCH_FILE = net_sim_launch_file
        self.LISTEN_ADDRESS = (address, get_port_from(address, True))
        self.keep_runing = Lock()

    def start(self: Sim) -> None:
        # Start simulated network
        if self.NET_PROC is None and not is_sim_network_running(self.LOCAL_IPS):
            self.NET_PROC = launch_sim_network(self.NET_SIM_LAUNCH_FILE, self.LOCAL_IPS)
        # Setup Pub/Sub topics
        self.register_ros_topics()
        # Launch listener and broadcaster threads
        self.executor: ThreadPoolExecutor = ThreadPoolExecutor(max_workers=2)
        self.keep_runing.acquire()
        self.thread_tasks = [
            self.executor.submit(self._listener),
            self.executor.submit(self._broadcaster),
        ]
        # Ensure all tasks are running
        for t in self.thread_tasks:
            while not t.running():
                print("NOT RUNNING")
                print("Done:", t.done())
                if t.done():
                    return
                time.sleep(QUICK_WAIT_TIMER)

    def stop(self: Sim) -> None:
        if self.NET_PROC is not None:
            terminate_sim_network(self.NET_PROC, self.LOCAL_IPS)
        # Signal threads to terminate
        self.keep_runing.release()
        # Wait for all thredads to stop
        self.executor.shutdown(wait=True)
        # Shutdown Pub/Sub channels
        self.unregister_ros_topics()

    def _listener(self: Sim) -> None:
        # Blocks untill kill_thread_event is set
        print("Starting listener at :", self.LISTEN_ADDRESS)
        server(self.LISTEN_ADDRESS, self.keep_runing)
        print("Finished listening at :", self.LISTEN_ADDRESS)

    # TODO: Fix broadcaster
    def _broadcaster(self: Sim) -> None:
        print("Starting broadcaster for :", self.LISTEN_ADDRESS[0])
        nic = get_interface_from_ip(self.LISTEN_ADDRESS[0])
        if nic == '':
            raise Exception("Broadcaster" + self.LISTEN_ADDRESS[0] + "could not retrieve a valid network interface!")
        while self.keep_runing.locked():
            for neighbor in self.get_reachable_ips(nic):
                send_messsage(nic=nic, destination=neighbor, message="hello world :D")
            time.sleep(BROADCAST_INTERVAL)
        print("Finished broadcasting for :", self.LISTEN_ADDRESS[0])

    def get_reachable_ips(self: Sim, nic: str) -> List[Tuple[str, int]]:
        neighbors = []
        for ip in self.LOCAL_IPS:
            if ip != self.LISTEN_ADDRESS[0]:
                destination = (ip, get_port_from(ip, True))
                # Create non-blocking socket, bound to the local address for broadcasting
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    sock.setsockopt(socket.SOL_SOCKET, 25, str(nic + '\0').encode('utf-8'))
                    sock.settimeout(RESPONSE_TIMEOUT)
                    try:
                        sock.connect(destination)
                        neighbors.append(sock.getpeername())
                        sock.close()
                    except socket.error as e:
                        print("Socket Error:", e)
                    sock.close()

        return neighbors

    def register_ros_topics(self: Sim) -> None:
        self.pub = rospy.Publisher(
            name=PUB_TOPIC,
            data_class=String,
            queue_size=10)

        # NOTE: The Subscriber callback only accepts two params (according to the docs)
        # rospy.Subscriber.callback: fn(msg, cb_args)
        # This forces us to use a lambda function to ignore the 'self'
        # source: http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Subscriber-class.html
        self.sub = rospy.Subscriber(
            name=SUB_TOPIC,
            data_class=String,
            callback=lambda msg: self.listen_handler(msg, cb_args=None))

    def unregister_ros_topics(self: Sim) -> None:
        if self.pub is not None:
            self.pub.unregister()
        if self.sub is not None:
            self.sub.unregister()

    def listen_handler(self: Sim, msg: any, cb_args: any) -> None:
        rospy.loginfo("GOT MESSAGE")
        print('got -> ', msg.data)


def is_sim_network_running(local_ips: List[str]) -> bool:
    num_devs = len(local_ips)
    out = str(check_output(["sudo", "-S", "ip", "rule", "list"]))
    if out.count('lookup') != num_devs * 2 + 3 or len(get_device_numbers()) != num_devs:
        return False
    # Ensure all ips are present
    for ip in local_ips:
        if out.count(ip) != 1:
            return False
    return True


# Returns ROS launch parent
def launch_sim_network(launch_file: str, local_ips: List[str]) -> roslaunch.parent.ROSLaunchParent:
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [launch_file],
        force_screen=False)
    launch.start()
    while not is_sim_network_running(local_ips):
        time.sleep(QUICK_WAIT_TIMER)
    return launch


def terminate_sim_network(launch: roslaunch.parent.ROSLaunchParent, local_ips: List[str]) -> None:
    launch.shutdown()
    for i in get_device_numbers():
        remove_net_tunnel(i)
    remove_net_rules()
    while is_sim_network_running(local_ips):
        time.sleep(QUICK_WAIT_TIMER)


def remove_net_tunnel(i: int) -> None:
    call(["sudo", "-S", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
    call(["sudo", "ip", "link", "delete", "dev", "tun" + str(i)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 101)])


def remove_net_rules() -> None:
    call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])


# # Create SIMPLEST form of:
# # listener and broadcaster

# # Simply send and recieve strings

# # Create a Message Interface

# # Create a dummyMessage to impliment it (sends a string as a payload)
