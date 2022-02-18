from __future__ import annotations
from typing import Tuple
from msg.message import Message
from msg.utils import extract_payload_id, request_merge, get_latest
from coms.constants import ENCODING
from nav_msgs.msg import OccupancyGrid
from ros_utils import numpy_to_ros
import numpy as np
import rospy


class Merge(Message):
    id = 0

    map: np.ndarray = None
    shape = 0, 0
    target: Tuple[str, int] = None
    source: Tuple[str, int] = None

    def __init__(self: Message,
                 map: np.ndarray,
                 source: Tuple[str, int] = None,
                 target: Tuple[str, int] = None) -> None:
        super().__init__()
        self.map: np.ndarray = map
        self.shape = map.shape
        self.source = source
        self.target = target

    def produce_payload(self: Message) -> bytes:
        map_bytes = self.map.tobytes()
        msg = f"{self.id}|{self.source}|{self.target}|{self.shape[0]}|{self.shape[1]}|{map_bytes}"  # noqa: E501
        return msg.encode(ENCODING)

    def consume_payload(self: Message, payload: bytes) -> Message:
        if extract_payload_id(payload) != self.id:
            raise Exception("Merge message can't consume the following payload\n{0}".format(payload))  # noqa: E501

        def dissect_tuple_str(s: str) -> Tuple[str, int]:
            if s == '()':
                return ()
            s = s.replace("'", '')
            s = s.replace('"', '')
            s = s.replace("(", '')
            s = s.replace(")", '')
            parts = s.split(',')
            return (parts[0], int(parts[1], 10))

        msg: str = payload.decode(ENCODING)
        parts = msg.split("|")

        source = dissect_tuple_str(parts[2])
        target = dissect_tuple_str(parts[3])
        shape = parts[4], parts[5]
        map_bytes = parts[6]

        map = np.frombuffer(map_bytes).reshape(shape)
        return Merge(map, source, target)

    # NOTICE: The Listener must call this function on the Merge message is recieves.
    # Do NOT call this method without the values msg.target, msg.source, msg.map
    # THIS METHOD WILL BE CALLED FROM THE ROS MODULE TO OBTAIN OWN LOCAL MAP
    def handle(self: Message, ros: rospy) -> Message:
        foreign_map: np.ndarray = self.map
        foreign_map = self.create_occupancy_msg(foreign_map)
        # send new map to map handler to perform merge
        request_merge(ros, foreign_map)
        # call NetSimMerger.request_merge() with foreign map
        local_map = get_latest(ros)
        # Construct Merge message as response
        return Merge(map=local_map, source=self.target, target=self.source)

    def on_failure() -> None:
        pass
