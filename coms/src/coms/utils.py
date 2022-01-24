import socket
import select
import yaml
from typing import List
from coms.constants import RESPONSE_TIMEOUT


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
