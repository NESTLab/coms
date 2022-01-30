from __future__ import annotations
from msg.message import Message


class Merge(Message):
    id = 0

    def __init__(self: Message) -> None:
        super().__init__()

    def produce_payload(self: Message) -> bytes:
        pass

    def consume_payload(self: Message, payload: bytes) -> Message:
        pass

    def handle(self: Message) -> any:
        pass

    def on_failure() -> None:
        pass
