from __future__ import annotations
import time
import kthread
import socket
from typing import List, Tuple
from coms.utils import Device, IndependentProcess, writable
from coms.constants import BROADCAST_INTERVAL, POSSIBLE_IPS, DISCOVERABLE_TIMEOUT, RESPONSE_TIMEOUT
from coms.map_subscriber import MapSubscriber
from nav_msgs.msg import OccupancyGrid


class Broadcaster(IndependentProcess):
    """ Client socket responsable for initiating conversation with remote listeners """
    def __init__(self: Broadcaster, address: Tuple, namespace: str, map_sub: MapSubscriber) -> None:
        super().__init__()
        self.address = address
        self.namespace = namespace
        self.map_sub = map_sub

    def start(self: Broadcaster) -> None:
        super().start(kthread.KThread(target=Broadcaster.broadcast, args=(self.address, self.namespace, self.map_sub,)))

    def stop(self: Broadcaster) -> None:
        super().stop()

    @staticmethod
    def broadcast(address: Tuple, namespace: str, map_sub: MapSubscriber) -> None:

        while True:
            print(address, "broadcasting")
            neighbors = Broadcaster.get_reachable_ips(address)
            for neighbor in neighbors:
                Broadcaster.handle_merge(
                    local_address=address,
                    listener_address=neighbor,
                    map_sub=map_sub,
                    num_retries=1)

            time.sleep(BROADCAST_INTERVAL)

    @staticmethod
    def get_reachable_ips(local_address: Tuple) -> List:
        """ Return a list of nearby IP addresses """
        neighbors = []
        for ip in POSSIBLE_IPS:
            if ip != local_address[0]:
                address = (ip, Device._get_port(ip, False))
                # Create non-blocking socket, bound to the local address for broadcasting
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.setblocking(False)
                sock.settimeout(DISCOVERABLE_TIMEOUT)
                sock.bind(local_address)
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

    @staticmethod
    def handle_merge(local_address: Tuple, listener_address: Tuple,
                     map_sub: MapSubscriber, num_retries: int = 2) -> None:
        while num_retries > 0:
            num_retries -= 1
            # Obtain latest map
            local_map: OccupancyGrid = map_sub.get_occupancy_grid()
            MapSubscriber.print_occupancy_grid(local_map)
            # raw_map = np.array(local_map.data, dtype=np.int8).tobytes()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setblocking(False)
            sock.settimeout(RESPONSE_TIMEOUT)
            sock.bind(local_address)
            try:
                # Send map to listener
                sock.connect(listener_address)

            except socket.error:
                continue

            sock.close()
