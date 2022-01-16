from __future__ import annotations
import socket
import kthread
from typing import Tuple
from coms.constants import MAX_CLIENTS
from coms.utils import IndependentProcess, readable
from coms.constants import RESPONSE_TIMEOUT


class Listener(IndependentProcess):
    """ Server socket responsible for responding to broadcaster """
    def __init__(self: Listener, address: Tuple, namespace: str) -> None:
        super().__init__()
        self.address = address
        self.namespace = namespace

    def start(self: Listener) -> None:
        super().start(kthread.KThread(target=Listener.listen, args=(self.address, self.namespace,)))

    def stop(self: Listener) -> None:
        super().stop()

    @staticmethod
    def listen(address: Tuple, namespace: str) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
        sock.setblocking(True)
        sock.bind(address)
        sock.listen(MAX_CLIENTS)

        while True:
            client_sock, client_addr = sock.accept()
            try:
                # Ensure client is reachable
                if not readable(client_sock, timeout=RESPONSE_TIMEOUT):
                    raise socket.error("Client socket not readable")

            except socket.error:
                continue

            client_sock.close()
