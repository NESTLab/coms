import socket
import select
import yaml
from typing import List
from coms.constants import RESPONSE_TIMEOUT, LISTENER_PORT_PREFIX, BROADCASTER_PORT_PREFIX


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


def get_port_from(ip: str, listener: bool) -> int:
    prefix: int = BROADCASTER_PORT_PREFIX
    if listener:
        prefix = LISTENER_PORT_PREFIX
    parts = ip.split('.')
    postfix = parts[len(parts) - 1]
    n = int(postfix, 10)
    if n < 10:
        p = int(prefix / 10) * 10
        return p + n
    p = int(prefix / 100) * 100
    return p + n
