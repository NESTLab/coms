import socket
import select
import yaml
import netifaces
from typing import List, Dict
from coms.constants import RESPONSE_TIMEOUT, ENCODING, STATIC_LISTENER_PORT, BROADCASTER_PORT_PREFIX, LISTENER_PORT_PREFIX # noqa: E501
from subprocess import check_output
from roslaunch.parent import ROSLaunchParent


def readable(sock: socket.socket, timeout: float = RESPONSE_TIMEOUT) -> bool:
    try:
        ready_to_read, _, _ = select.select([sock], [], [], timeout)
        return len(ready_to_read) > 0
    except Exception:
        return False


def writable(sock: socket.socket, timeout: float = RESPONSE_TIMEOUT) -> bool:
    try:
        _, ready_to_write, _ = select.select([], [sock], [], timeout)
        return len(ready_to_write) > 0
    except Exception:
        return False


def get_ip_list(yaml_config_path: str) -> List[str]:
    with open(yaml_config_path, 'r') as file:
        return yaml.safe_load(file)['ip_list']


# Retrieves the name of the network interface bound to the given ip address.
# If no interface has the ip address given, an empty string will be returned.
def get_interface_from_ip(ip: str) -> str:
    af_inet: int = netifaces.AF_INET
    ifaces: List[str] = netifaces.interfaces()
    for interface in ifaces:
        iface_info: Dict = netifaces.ifaddresses(interface)
        if af_inet not in iface_info:
            continue
        ip_info_list: List = iface_info[af_inet]
        if len(ip_info_list) == 0:
            continue
        ip_info: Dict = ip_info_list[0]
        if 'addr' not in ip_info:
            continue
        addr: str = ip_info['addr']
        if addr == ip:
            return interface
    # Interface not found
    return ''


# Return list of tunnel device numbers
# If the OS had the following tunnels:  tun0, tun1, tun2
# get_device_numbers would return:      [0, 1, 2]
def get_device_numbers() -> List[int]:
    devices: List[int] = []
    out = check_output(["sudo", "-S", "ip", "tuntap", "list", "dev", "mode", "tun"])
    for line in out.splitlines():
        parts = line.decode(ENCODING).split(':')
        dev_number = int(parts[0][3], 10)
        devices.append(dev_number)
    return devices


# Returns TCP socket bound to the given ip address
def gen_bound_socket(ip: str) -> socket.socket:
    iface: str = get_interface_from_ip(ip)
    if iface == '':
        raise Exception("Unable to find network interface with given IP " + ip)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, 25, str(iface + '\0').encode('utf-8'))
    return sock


# IMPORTANT: This function is used as a function to manage ports for listener and broadcaster.
# Therefore, if we want to change the ports in any which way, we can change it once, right here!
def get_port_from(ip: str, listener: bool) -> int:
    # prefix: int = BROADCASTER_PORT_PREFIX
    # if listener:
    #     prefix = LISTENER_PORT_PREFIX
    # parts = ip.split('.')
    # postfix = parts[len(parts) - 1]
    # n = int(postfix, 10)
    # if n < 10:
    #     p = int(prefix / 10) * 10
    #     return p + n
    # p = int(prefix / 100) * 100
    # return p + n
    return STATIC_LISTENER_PORT


def start_roscore() -> ROSLaunchParent:
    parent = ROSLaunchParent("", [], is_core=True)     # run_id can be any string
    parent.start()
    return parent


def stop_roscore(parent: ROSLaunchParent) -> None:
    parent.shutdown()
