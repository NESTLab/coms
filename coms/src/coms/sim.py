from __future__ import annotations
import time
import socket
import threading
from typing import List, Tuple
from subprocess import check_output, call
from coms.constants import QUICK_WAIT_TIMER, STATIC_LISTENER_PORT, ENCODING, CATKIN_WS, NET_CONFIG, DISCOVERABLE_TIMEOUT
from coms.utils import writable, get_ip_list, gen_bound_socket, get_device_numbers
from concurrent.futures import ThreadPoolExecutor, Future
import roslaunch


class Sim():
    LOCAL_IPS: List[str] = []                               # List of ip addresses defined in configuration file
    LISTEN_ADDRESS: Tuple[str, int] = ()                    # Address bound to the listener's TCP socket server
    NIC: str = ""                                           # Network Interface Card - Retrieved from ip address
    thread_executor: ThreadPoolExecutor = None              # Executor for managing threaded operations
    thread_tasks: List[Future] = []                         # List of Future objects for obtaining of thread stats
    kill_thread_event: threading.Event = threading.Event()  # Event for signaling when threaded tasks should terminate
    NET_PROC: roslaunch.parent.ROSLaunchParent = None       # ROS specific launch object - for starting sim network
    NET_SIM_LAUNCH_FILE: str = ""                           # Path to sim network launch file

    def __init__(self: Sim, address: str, net_sim_launch_file: str) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, NET_CONFIG), shell=True)
        self.LOCAL_IPS = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.NET_SIM_LAUNCH_FILE = net_sim_launch_file
        self.LISTEN_ADDRESS = (address, STATIC_LISTENER_PORT)
        self.thread_executor = ThreadPoolExecutor(max_workers=2)

    def start(self: Sim) -> None:
        if self.NET_PROC is None:
            self.NET_PROC = launch_sim_network(self.NET_SIM_LAUNCH_FILE, self.LOCAL_IPS)
        self.thread_tasks = []
        self.thread_tasks.append(self.thread_executor.submit(self._listener))
        self.thread_tasks.append(self.thread_executor.submit(self._broadcaster))
        # Ensure all tasks are running
        for t in self.thread_tasks:
            while not t.running():
                time.sleep(QUICK_WAIT_TIMER)

    def stop(self: Sim) -> None:
        if self.NET_PROC is not None:
            terminate_sim_network(self.NET_PROC, self.LOCAL_IPS)
        # Signal threads to terminate
        self.kill_thread_event.set()
        # Wait for all thredads to stop
        self.thread_executor.shutdown(wait=True)

    def _listener(self: Sim) -> None:
        while not self.kill_thread_event.is_set():
            time.sleep(0.5)

    def _broadcaster(self: Sim) -> None:
        while not self.kill_thread_event.is_set():
            time.sleep(0.5)

    def get_reachable_ips(self: Sim, self_address: Tuple[str, int]) -> List[Tuple[str, int]]:
        neighbors = []
        for ip in self.LOCAL_IPS:
            if ip != self_address[0]:
                address = (ip, STATIC_LISTENER_PORT)
                # Create non-blocking socket, bound to the local address for broadcasting
                sock = gen_bound_socket(self_address)

                # sock.setblocking(False)
                sock.settimeout(DISCOVERABLE_TIMEOUT)

                try:
                    # Attempt to connect to remote listener
                    sock.connect(address)
                    if writable(sock, timeout=DISCOVERABLE_TIMEOUT):
                        neighbors.append(address)
                except socket.error:
                    # Connection timed out
                    pass
                sock.close()
        return neighbors


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
