from __future__ import annotations
from ast import arg
import time
import socket
import threading
from subprocess import Popen, check_output, DEVNULL, call
from typing import List, Tuple
from coms.constants import BROADCASTER_PORT, LISTENER_PORT, MAX_CLIENTS, RESPONSE_TIMEOUT, BROADCAST_INTERVAL, ENCODING, CATKIN_WS, NET_CONFIG, DISCOVERABLE_TIMEOUT
from coms.utils import writable, readable, get_ip_list


class Sim():
    LOCAL_IPS: List[str] = []
    L_PORT: int = LISTENER_PORT
    L_ADDRESS: str = ""
    B_PORT: int = BROADCASTER_PORT
    B_ADDRESS: str = ""
    stay_alive: bool = True
    threads: List[threading.Thread] = []
    kill_thread_event: threading.Event = threading.Event()
    NET_PROC: Popen = None

    def __init__(self: Sim, listen_address: str, broadcast_address: str) -> None:
        path_to_config = check_output("find {0} -type f -name '{1}'".format(CATKIN_WS, NET_CONFIG), shell=True)
        self.LOCAL_IPS = get_ip_list(path_to_config.decode(ENCODING).strip())
        self.L_ADDRESS = listen_address
        self.B_ADDRESS = broadcast_address

    def start(self: Sim) -> None:
        if not is_sim_network_running():
            self.NET_PROC = launch_sim_network()

        self.threads = []
        self.threads.append(threading.Thread(
            target=self.start_listener))
        self.threads.append(threading.Thread(
            target=self.start_broadcaster))
        self._start_threads()

    def stop(self: Sim) -> None:
        self._stop_threads()
        if self.NET_PROC is not None:
            terminate_sim_network(self.NET_PROC)

    def _start_threads(self: Sim) -> None:
        for t in self.threads:
            t.start()
            while not t.is_alive():
                time.sleep(0.1)

    def _stop_threads(self: Sim) -> None:
        self.kill_thread_event.set()
        for t in self.threads:
            t.join()

    def start_listener(self: Sim) -> None:
        address: Tuple[str, int] = (self.L_ADDRESS, self.L_PORT)
        sock = gen_bound_socket(address)
        sock.listen(MAX_CLIENTS)

        while not self.kill_thread_event.is_set():
            client_sock, client_addr = sock.accept()
            try:
                # Ensure client is reachable
                if not readable(client_sock, timeout=RESPONSE_TIMEOUT):
                    raise socket.error("Client socket not readable")
            except socket.error:
                continue

            print("Incoming message from:", client_addr)

            client_sock.close()

    def start_broadcaster(self: Sim) -> None:
        address: Tuple[str, int] = (self.B_ADDRESS, self.B_PORT)
        while not self.kill_thread_event.is_set():
            neighbors = self.get_reachable_ips(address)
            for neighbor in neighbors:
                send(address, neighbor)
            time.sleep(BROADCAST_INTERVAL)

    def get_reachable_ips(self: Sim, self_address: Tuple[str, int]) -> List[Tuple[str, int]]:
        neighbors = []
        for ip in self.LOCAL_IPS:
            if ip != self_address[0]:
                address = (ip, self.L_PORT)
                # Create non-blocking socket, bound to the local address for broadcasting
                sock = gen_bound_socket(self_address)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setblocking(False)
                sock.settimeout(DISCOVERABLE_TIMEOUT)
                try:
                    # Attempt to connect to remote listener
                    sock.connect(address)
                    if writable(sock, timeout=DISCOVERABLE_TIMEOUT):
                        neighbors.append(address)
                except socket.error:
                    # Connection timed out
                    continue
                sock.close()
        return neighbors


def is_sim_network_running() -> bool:
    return len(get_device_numbers()) > 0


# Returns PID of the launch process
def launch_sim_network() -> Popen:
    ros_setup_script = "/opt/ros/noetic/setup.sh"
    devel_setup_script = CATKIN_WS + "/devel/setup.sh"
    command = "roslaunch example gazebo.launch"
    # This launch file explicitly creates 3 network devices
    cmd = "sh {0} && sh {1} && {2}".format(ros_setup_script, devel_setup_script, command)
    proc = Popen(cmd, stdout=DEVNULL, stderr=DEVNULL, shell=True)
    # Wait for the OS to create all network devices
    while not is_sim_network_running():
        time.sleep(0.1)
    return proc


def terminate_sim_network(proc: Popen) -> None:
    while is_sim_network_running():
        while len(get_device_numbers()) != 0:
            devices = get_device_numbers()
            print(devices)
            proc.kill()
            for dev in devices:
                remove_net_tunnel(dev)
        remove_net_rules()
        time.sleep(0.1)


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
    sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_STREAM, laddr=address)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
    sock.setblocking(True)
    sock.bind(address)
    return sock


def send(f: Tuple[str, int], t: Tuple[str, int], num_retries: int = 2) -> None:
    while num_retries > 0:
        num_retries -= 1

        sock = gen_bound_socket(f)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            continue

        sock.close()


# # Create SIMPLEST form of:
# # listener and broadcaster

# # Simply send and recieve strings

# # Create a Message Interface

# # Create a dummyMessage to impliment it (sends a string as a payload)
