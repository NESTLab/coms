from __future__ import annotations
import hashlib
import socket
import select
import netifaces
import rospy
from kthread import KThread
from coms.constants import RESPONSE_TIMEOUT, PACKET_BUFFER_SIZE
from abc import ABC, abstractmethod


class Device(ABC):
    @abstractmethod
    def __init__(self: Device, network_interface: str) -> None:
        super().__init__()
        self.network_interface = network_interface
        self.local_ip_addr = Device._get_local_ip(network_interface)

    @abstractmethod
    def start(self: Device) -> None:
        pass

    @abstractmethod
    def stop(self: Device) -> None:
        pass

    @staticmethod
    def _get_local_ip(network_interface: str) -> str:
        return netifaces.ifaddresses(network_interface)[netifaces.AF_INET][0]['addr']

    @staticmethod
    def _get_port(ip_addr: str, outgoing: bool) -> int:
        """
        Ports are determined by the last digits of an IP
        Examples:   IP: 192.168.0.2    Port: 9002
                    IP: 192.168.0.11   Port: 9011
        """
        partial_port = '90'
        if outgoing:
            partial_port = '80'
        tail = ip_addr.split('.')[3]
        if len(tail) == 1:
            return int(partial_port + '0' + tail, 10)
        # For addresses like XXX.XXX.X.10 
        return int(partial_port + tail, 10)


class IndependentProcess(ABC):
    @abstractmethod
    def __init__(self: IndependentProcess) -> None:
        self.main_process: KThread = None

    @abstractmethod
    def start(self: IndependentProcess, thread: KThread) -> None:
        self.main_process = thread
        self.main_process.start()

    @abstractmethod
    def stop(self) -> None:
        if self.main_process.is_alive():
            self.main_process.terminate()


def b_to_mb(b: int) -> float:
    return b/1000000


# Consistant hash function
def uhash(data: any) -> str:
    h = hashlib.sha256()
    h.update(str(data).encode())
    return h.digest().hex()


def readable(sock: socket.socket, timeout: int = RESPONSE_TIMEOUT) -> bool:
    ready_to_read, _, _ = select.select([sock], [], [], timeout)
    return len(ready_to_read) > 0


def writable(sock: socket.socket, timeout: int = RESPONSE_TIMEOUT) -> bool:
    _, ready_to_write, _ = select.select([], [sock], [], timeout)
    return len(ready_to_write) > 0


def write_all(sock: socket.socket, message: bytes, message_length: int) -> None:
    if not writable(sock, timeout=RESPONSE_TIMEOUT):
        raise socket.error("Socket not writable")

    if len(message) != message_length:
        rospy.logwarn("write_all recieved message of different length than specified. Message:{}".format(message))

    bytes_sent = 0
    while bytes_sent < message_length:
        sent = sock.send(message[bytes_sent:], message_length)
        if sent == 0:
            # Conection lost
            raise socket.error("Socket connection lost during write_all")
        bytes_sent += sent


def read_all(sock: socket.socket, message_length: int) -> bytes:
    if not readable(sock, timeout=RESPONSE_TIMEOUT):
        raise socket.error("Socket not readable")

    packets = []
    bytes_recieved = 0
    while bytes_recieved < message_length:
        packet = sock.recv(min(message_length - bytes_recieved, PACKET_BUFFER_SIZE))
        if packet == b'':
            # Conection lost
            raise socket.error("Socket connection lost during read_all")
        packets.append(packet)
        bytes_recieved += len(packet)
    return b''.join(packets)
