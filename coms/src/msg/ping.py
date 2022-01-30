from __future__ import annotations
from msg.message import Message
from msg.utils import extract_payload_id
from coms.constants import ENCODING
from typing import Tuple


class Ping(Message):
    id = 1
    source: Tuple[str, int] = ()
    destination: Tuple[str, int] = ()

    def __init__(self: Message, source: Tuple[str, int] = (), destination: Tuple[str, int] = ()) -> None:
        super().__init__()
        self.source = source
        self.destination = destination

    def produce_payload(self: Ping) -> bytes:
        msg: str = "{0}|{1}|{2}".format(self.id, self.source, self.destination)
        return msg.encode(ENCODING)

    def consume_payload(self: Ping, payload: bytes) -> Message:
        if extract_payload_id(payload) != self.id:
            raise Exception("Ping message can't consume the following payload\n{0}".format(payload))
        msg: str = payload.decode(ENCODING)

        def disect_tuple_str(s: str) -> Tuple[str, int]:
            if s == '()':
                return ()
            s = s.replace("'", '')
            s = s.replace('"', '')
            s = s.replace("(", '')
            s = s.replace(")", '')
            parts = s.split(',')
            return (parts[0], int(parts[1], 10))

        parts = msg.split('|')
        src = disect_tuple_str(parts[1])
        dest = disect_tuple_str(parts[2])
        return Ping(src, dest)

    def handle(self: Ping) -> None:
        print("PING from {0} to {1}".format(self.source, self.destination))

    def on_failure() -> None:
        pass
