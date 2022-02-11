#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid
from mapmerge.ros_utils import pgm_to_numpy, numpy_to_ros

class TestGMAPPING:
    def __init__(self):
        # GET PARAMS
        self.robot_name = rospy.get_param('~robot_name', 'tb0')
        self.map_service = rospy.get_param('~map_service', 'dynamic_map')

        # DATA
        self.seq = 0
        self.map = np.eye(100, dtype=np.int16)
        self.data_path = "/root/catkin_ws/src/coms/testData" # avert your eyes, static path!
        self.map = pgm_to_numpy(self.data_path + "/full_map.pgm")

        # INIT ROS
        rospy.init_node(f'{self.robot_name}_test_GMAPPING')
        self.map_service = rospy.Service(self.map_service, GetMap, self.serve_map)

        # keep the process going
        self.rate = rospy.Rate(10)
        rospy.spin()

    def serve_map(self, _):
        """
        serves a dummy map. This can be configured to pass testcases through the rosnodes
        """
        self.seq += 1
        occ = OccupancyGrid()
        occ.header.seq = self.seq
        occ.data = list(numpy_to_ros(self.map.flatten()))
        return GetMapResponse(occ)



if __name__ == "__main__":
    TA = TestGMAPPING()