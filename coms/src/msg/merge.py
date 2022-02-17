from __future__ import annotations
from typing import Tuple
from msg.message import Message
from msg.utils import extract_payload_id
from coms.constants import ENCODING
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from coms.srv import MergeMap
from ros_utils import ros_to_numpy, numpy_to_ros
from keypoint_merge import sift_mapmerge
import numpy as np
import rospy

class Merge(Message):
    id = 0

    map: np.ndarray = None
    shape = 0, 0
    target: Tuple[str, int] = None
    source: Tuple[str, int] = None

    def __init__(self: Message, map: np.ndarray, source: Tuple[str, int] = None, target: Tuple[str, int] = None) -> None:
        super().__init__()
        self.map: np.ndarray = map
        self.shape = map.shape
        self.source = source
        self.target = target
        

    def produce_payload(self: Message) -> bytes:
        map_bytes = self.map.tobytes()
        msg = f"{self.id}|{self.source}|{self.target}|{self.shape[0]}|{self.shape[1]}|{map_bytes}"
        return msg.encode(ENCODING)

    def consume_payload(self: Message, payload: bytes) -> Message:
        if extract_payload_id(payload) != self.id:
            raise Exception("Merge message can't consume the following payload\n{0}".format(payload))
        
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
        self.request_merge(ros, foreign_map)
        
        # call NetSimMerger.request_merge() with foreign map
        local_map = self.get_latest(ros)
 
        # Construct Merge message as response
        return Merge(map = local_map, source = self.target, target = self.source)

    def on_failure() -> None:
        pass

    def create_occupancy_msg(self, map: np.ndarray, seq: int = 0) -> OccupancyGrid:
        occ = OccupancyGrid()
        occ.header.seq = seq
        occ.data = tuple(numpy_to_ros(map.flatten()))
        return occ

    def request_merge(self: Merge, ros: rospy, foriegn_map: OccupancyGrid) -> None:
        """
        Requests a merge from the specified ros service {merge_service}.
        """
        robot_name = ros.get_name()
        ros.wait_for_service(f'{robot_name}/merge')
        attempts = 0
        max_attemps = 10
        try:
            req = foriegn_map
            merge_service = ros.ServiceProxy(f'{robot_name}/merge', MergeMap)
            return merge_service(req)
        except ros.ServiceException as e:
            attempts += 1
            ros.loginfo("service call failed: %s" % e)
            if attempts >= max_attemps:
                return False
        return False


    def get_latest(self: Merge, ros: rospy) -> None:
        """
        gets the latest map from map service
        """
        robot_name = ros.get_name()

        ros.wait_for_service(f"{robot_name}/get_map")
        while 1:
            try:
                map_service = ros.ServiceProxy(f"{robot_name}/get_map", GetMap)
                return ros_to_numpy(map_service().map)
            except ros.ServiceException as e:
                ros.loginfo("service call failed: %s" % e)
            ros.Rate(10).sleep()