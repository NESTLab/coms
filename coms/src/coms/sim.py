from __future__ import annotations
from concurrent.futures import thread
import time
import socket
import threading
from subprocess import check_output, call
from typing import List, Tuple
from coms.constants import MAX_CLIENTS, RESPONSE_TIMEOUT, BROADCAST_INTERVAL, ENCODING, CATKIN_WS, NET_CONFIG, DISCOVERABLE_TIMEOUT # noqa: E501
from coms.utils import writable, readable, get_ip_list, get_port_from
import roslaunch
from concurrent.futures import ThreadPoolExecutor, Future
import rospy


class Sim():
    LOCAL_IPS: List[str] = []
    LISTEN_ADDRESS: Tuple[str, int] = ()
    BROADCAST_ADDRESS: Tuple[str, int] = ()
    thread_executor: ThreadPoolExecutor = None
    thread_tasks: List[Future] = []
    kill_thread_event: threading.Event = threading.Event()
    NET_PROC: roslaunch.parent.ROSLaunchParent = None
    NET_SIM_LAUNCH_FILE: str = ""

    def __init__(self: Sim, address: str, net_sim_launch_file: str) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, NET_CONFIG), shell=True)
        self.LOCAL_IPS = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.NET_SIM_LAUNCH_FILE = net_sim_launch_file
        self.LISTEN_ADDRESS = (address, get_port_from(ip=address, listener=True))
        self.BROADCAST_ADDRESS = (address, get_port_from(ip=address, listener=False))
        self.thread_executor = ThreadPoolExecutor(max_workers=2)

    def start(self: Sim) -> None:
        if self.NET_PROC is None:
            self.NET_PROC = launch_sim_network(self.NET_SIM_LAUNCH_FILE, self.LOCAL_IPS)
        self.thread_tasks = []
        self.thread_tasks.append(self.thread_executor.submit(self._listener))
        self.thread_tasks.append(self.thread_executor.submit(self._broadcaster))
        for t in self.thread_tasks:
            while not t.running():
                time.sleep(0.01)

    def stop(self: Sim) -> None:
        if self.NET_PROC is not None:
            terminate_sim_network(self.NET_PROC, self.LOCAL_IPS)
        self.kill_thread_event.set()
        self.thread_executor.shutdown(wait=True)

    def _listener(self: Sim) -> None:
        sock = gen_bound_socket(self.LISTEN_ADDRESS)
        sock.settimeout(DISCOVERABLE_TIMEOUT)
        sock.listen(MAX_CLIENTS)

        while not self.kill_thread_event.is_set():
            client_sock, client_addr = sock.accept()
            try:
                # Ensure client is reachable
                if not readable(client_sock, timeout=RESPONSE_TIMEOUT):
                    raise socket.error("Client socket not readable")
                print("Incoming message from:", client_addr)
            except socket.timeout:
                pass
            except socket.error:
                pass
            client_sock.close()
        sock.close()

    def _broadcaster(self: Sim) -> None:
        while not self.kill_thread_event.is_set():
            neighbors = self.get_reachable_ips(self.BROADCAST_ADDRESS)
            for neighbor in neighbors:
                send(self.BROADCAST_ADDRESS, neighbor)
            time.sleep(BROADCAST_INTERVAL)

    def get_reachable_ips(self: Sim, self_address: Tuple[str, int]) -> List[Tuple[str, int]]:
        neighbors = []
        for ip in self.LOCAL_IPS:
            if ip != self_address[0]:
                address = (ip, get_port_from(ip=ip, listener=True))
                # Create non-blocking socket, bound to the local address for broadcasting
                sock = gen_bound_socket(self_address)
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
        time.sleep(0.05)
    return launch


def terminate_sim_network(launch: roslaunch.parent.ROSLaunchParent, local_ips: List[str]) -> None:
    launch.shutdown()
    for i in get_device_numbers():
        remove_net_tunnel(i)
    remove_net_rules()
    while is_sim_network_running(local_ips):
        time.sleep(0.05)


def get_device_numbers() -> List[int]:
    devices: List[int] = []
    out = check_output(["sudo", "-S", "ip", "tuntap", "list", "dev", "mode", "tun"])
    for line in out.splitlines():
        parts = line.decode(ENCODING).split(':')
        dev_number = int(parts[0][3], 10)
        devices.append(dev_number)
    return devices


def remove_net_tunnel(i: int) -> None:
    call(["sudo", "-S", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
    call(["sudo", "ip", "link", "delete", "dev", "tun" + str(i)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 101)])


def remove_net_rules() -> None:
    call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])


def gen_bound_socket(address: Tuple[str, int]) -> socket.socket:
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
    sock.setblocking(True)
    sock.bind(address)
    return sock


def send(f: Tuple[str, int], t: Tuple[str, int], num_retries: int = 2) -> None:
    while num_retries > 0:
        num_retries -= 1

        sock = gen_bound_socket(f)
        sock.setblocking(False)
        sock.settimeout(RESPONSE_TIMEOUT)
        sock.bind(f)
        try:
            # Send map to listener
            sock.connect(t)
            if not readable(sock):
                raise socket.error("Listener socket not readable")

            sock.sendall(b'Hello world')
            num_retries = 0

        except socket.error:
            pass

        sock.close()


# # Create SIMPLEST form of:
# # listener and broadcaster

# # Simply send and recieve strings

# # Create a Message Interface

# # Create a dummyMessage to impliment it (sends a string as a payload)
