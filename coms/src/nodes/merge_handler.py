#!/usr/bin/env python3
import rospy
from coms.srv import MergeMap, MergeMapResponse, MergeMapRequest
from nav_msgs.srv import GetMap, GetMapResponse, GetMapRequest
from nav_msgs.msg import OccupancyGrid
from mapmerge.keypoint_merge import orb_mapmerge
import numpy as np


class MergeHandler:
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'tb0')
        self.map_size = rospy.get_param('~map_size', 100)

        self.latest_map = np.array([])
        self.seq = 0

        rospy.init_node(f"{self.robot_name}_MergeHandler")
        self.merge_service = rospy.Service(self.robot_name + "/merge", MergeMap, self.merge_new)
        self.get_map_service = rospy.Service(self.robot_name + "/get_map", GetMap, self.serve_map)
        self.map_publisher = rospy.Publisher(f'{self.robot_name}/map', OccupancyGrid, queue_size=10)

        self.rate = rospy.Rate(10)  # 10hz

    def run(self) -> None:
        rospy.loginfo(f'{self.robot_name} merge handler node starting')
        while not rospy.is_shutdown():
            self.publish_latest()
            self.rate.sleep()
        rospy.loginfo(f'{self.robot_name} merge handler node shutting down')

    def publish_latest(self) -> None:
        occ = self.create_occupancy_msg(self.seq)
        self.seq += 1
        self.map_publisher.publish(occ)

    def merge_new(self, req: MergeMapRequest) -> MergeMapResponse:
        new_map = req.map.data
        try:
            merged = orb_mapmerge(new_map, self.latest_map)
            # TODO checks and maybe a lock? depends on the rospy callback threading
            self.latest_map = merged
            return True
        except Exception as e:
            rospy.loginfo(f"Could not merge maps: {e}")
        return False

    def serve_map(self, _) -> GetMapResponse:
        """
        serves the latest map
        """
        occ = self.create_occupancy_msg(self.seq)
        return GetMapResponse(occ)

    def create_occupancy_msg(self, seq: int) -> OccupancyGrid:
        occ = OccupancyGrid()
        occ.header.seq = seq
        occ.data = tuple(self.latest_map.flatten())
        return occ

if __name__ == "__main__":
    MH = MergeHandler()
    MH.run()
