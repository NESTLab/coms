from __future__ import annotations
from abc import ABC, abstractmethod


class Message(ABC):

    retries = 5
    timeout = 4

    def __init__(self: Message) -> None:
        pass

    @abstractmethod
    def produce_payload(self: Message) -> bytes:
        pass

    @abstractmethod
    def consume_payload(self: Message, payload: bytes) -> Message:
        pass

    @abstractmethod
    def handle(self: Message) -> any:
        pass

    @abstractmethod
    def on_failure() -> None:
        pass
