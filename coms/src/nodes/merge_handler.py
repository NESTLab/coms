#!/usr/bin/env python3
import rospy
# from coms.srv import MergeMap, MergeMapResponse, MergeMapRequest
from nav_msgs.srv import GetMap, GetMapResponse, GetMapRequest
from nav_msgs.msg import OccupancyGrid
from mapmerge.keypoint_merge import orb_mapmerge
from mapmerge.ros_utils import pgm_to_numpy, numpy_to_ros, ros_to_numpy
import numpy as np
import matplotlib.pyplot as plt

class MergeHandler:
    def __init__(self):
        # self.robot_name = rospy.get_param('~robot_name', 'tb0')
        # self.map_size = rospy.get_param('~map_size', 100)

        #self.latest_map = np.array([])
        self.seq = 0
        self.data_path = "/home/connor/Downloads" # avert your eyes, static path!
        self.latest_map = pgm_to_numpy(self.data_path + "/map_part0.pgm")
        self.test_map = pgm_to_numpy(self.data_path + "/full_map.pgm")

        fig, axes = plt.subplots(3, 2)

        axes[0][0].imshow(self.latest_map, cmap="gray")
        print(np.unique(self.latest_map))
        axes[0][1].imshow(self.test_map, cmap="gray")
        print(np.unique(self.test_map))

        self.latest_map = numpy_to_ros(self.latest_map)
        self.test_map = numpy_to_ros(self.test_map)

        axes[1][0].imshow(self.latest_map, cmap="gray")
        print(np.unique(self.latest_map))
        axes[1][1].imshow(self.test_map, cmap="gray")
        print(np.unique(self.test_map))

        self.latest_map = ros_to_numpy(self.latest_map)
        self.test_map = ros_to_numpy(self.test_map)

        axes[2][0].imshow(self.latest_map, cmap="gray")
        print(np.unique(self.latest_map))
        axes[2][1].imshow(self.test_map, cmap="gray")
        print(np.unique(self.test_map))
        plt.show()

        # print(orb_mapmerge(self.latest_map, self.test_map))
        # breakpoint()

        # rospy.init_node(f"{self.robot_name}_MergeHandler")
        # self.merge_service = rospy.Service(self.robot_name + "/merge", MergeMap, self.merge_new)
        # self.get_map_service = rospy.Service(self.robot_name + "/get_map", GetMap, self.serve_map)
        # self.map_publisher = rospy.Publisher(f'{self.robot_name}/map', OccupancyGrid, queue_size=10)

        # self.rate = rospy.Rate(10)  # 10hz

    def run(self) -> None:
        rospy.loginfo(f'{self.robot_name} merge handler node starting')
        while not rospy.is_shutdown():
            self.publish_latest()
            self.rate.sleep()
        rospy.loginfo(f'{self.robot_name} merge handler node shutting down')

    # def publish_latest(self) -> None:
    #     occ = self.create_occupancy_msg(self.seq)
    #     self.seq += 1
    #     self.map_publisher.publish(occ)

    # def merge_new(self, req: MergeMapRequest) -> MergeMapResponse:
    #     new_map = np.array(req.map.data)
    #     try:
    #         merged = orb_mapmerge(new_map, self.latest_map)
    #         # TODO checks and maybe a lock? depends on the rospy callback threading
    #         self.latest_map = merged
    #         return True
    #     except Exception as e:
    #         rospy.loginfo(f"Could not merge maps: {e}")
    #     return False

    # def serve_map(self, _) -> GetMapResponse:
    #     """
    #     serves the latest map
    #     """
    #     occ = self.create_occupancy_msg(self.seq)
    #     return GetMapResponse(occ)

    # def create_occupancy_msg(self, seq: int) -> OccupancyGrid:
    #     occ = OccupancyGrid()
    #     occ.header.seq = seq
    #     occ.data = tuple(self.latest_map.flatten())
    #     return occ

if __name__ == "__main__":
    MH = MergeHandler()
    MH.run()
