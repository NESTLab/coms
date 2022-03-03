import unittest
import os
import pathlib
from coms.sim import launch_sim_network, terminate_sim_network
from coms.utils import get_port_from, get_interface_from_ip
from coms.server import server, send_messsage
from msg.ping import Ping
from msg.message import Message
from threading import Thread, Lock
from typing import Tuple
from unittest.mock import patch, call

TEST_DIR = pathlib.Path(__file__).parent.absolute()
COMS_DIR = TEST_DIR.parent.absolute()
PROJECT_BASE_DIR = COMS_DIR.parent.absolute()
DEFAULT_NET_SIM_LAUNCH_FILE = os.path.join(PROJECT_BASE_DIR, "ros-net-sim/example/launch/testing.launch")
LAUNCH_CONFIG_LOCAL_IPS = ["192.168.0.1", "192.168.0.2", "192.168.0.3"]


class TestPing(unittest.TestCase):

    def test_produce_payload(self: unittest.TestCase) -> None:
        p = Ping()
        self.assertEqual(p.produce_payload(), b"1|()|()")
        p.source = ("192.168.0.1", 1234)
        self.assertEqual(p.produce_payload(), b"1|('192.168.0.1', 1234)|()")
        p.destination = ("192.168.5.3", 8081)
        self.assertEqual(p.produce_payload(), b"1|('192.168.0.1', 1234)|('192.168.5.3', 8081)")
        p = Ping(source=('192.168.8.8', 5544), destination=('192.168.0.3', 6261))
        self.assertEqual(p.produce_payload(), b"1|('192.168.8.8', 5544)|('192.168.0.3', 6261)")

    def test_consume_payload(self: unittest.TestCase) -> None:
        p = Ping()
        empty_payload: bytes = b"1|()|()"
        got: Ping = p.consume_payload(empty_payload)
        expect = Ping()
        self.assertEqual(got.id, expect.id)
        self.assertEqual(got.source, expect.source)
        self.assertEqual(got.destination, expect.destination)
        empty_payload: bytes = b"1|('192.168.0.1', 1234)|()"
        got: Ping = p.consume_payload(empty_payload)
        expect = Ping(source=('192.168.0.1', 1234))
        self.assertEqual(got.id, expect.id)
        self.assertEqual(got.source, expect.source)
        self.assertEqual(got.destination, expect.destination)
        empty_payload: bytes = b"1|('192.168.8.8', 5544)|('192.168.0.3', 6261)"
        got: Ping = p.consume_payload(empty_payload)
        expect = Ping(source=('192.168.8.8', 5544), destination=('192.168.0.3', 6261))
        self.assertEqual(got.id, expect.id)
        self.assertEqual(got.source, expect.source)
        self.assertEqual(got.destination, expect.destination)

    def test_handle(self: unittest.TestCase) -> None:
        with patch('msg.ping.print') as mock_print:
            Ping().handle()
            self.assertEqual(mock_print.call_count, 1)
            self.assertEqual(mock_print.call_args, call("PING from () to ()"))
            Ping(source=('192.168.8.8', 5544), destination=('192.168.0.3', 6261)).handle()
            self.assertEqual(mock_print.call_count, 2)
            self.assertEqual(mock_print.call_args, call("PING from ('192.168.8.8', 5544) to ('192.168.0.3', 6261)"))

    def test_transmission(self: unittest.TestCase) -> None:
        launch = launch_sim_network(DEFAULT_NET_SIM_LAUNCH_FILE, LAUNCH_CONFIG_LOCAL_IPS)
        lock = Lock()
        lock.acquire()
        addr: Tuple[str, int] = (
            LAUNCH_CONFIG_LOCAL_IPS[0],
            get_port_from(ip=LAUNCH_CONFIG_LOCAL_IPS[0], listener=True))
        t = Thread(target=server, args=[addr, lock])
        t.start()

        msg: Message = Ping()
        nic: str = get_interface_from_ip(LAUNCH_CONFIG_LOCAL_IPS[1])
        dest: Tuple[str, int] = addr
        send_messsage(nic, dest, msg)

        lock.release()
        terminate_sim_network(launch, LAUNCH_CONFIG_LOCAL_IPS)


if __name__ == '__main__':
    unittest.main()
