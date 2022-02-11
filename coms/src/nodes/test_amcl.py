#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid


class TestAMCL:
    def __init__(self):
        # GET PARAMS
        self.robot_name = rospy.get_param('~robot_name', 'tb0')

        # DATA
        self.map = np.array([])

        # INIT ROS
        rospy.init_node(f'{self.robot_name}_test_AMCL')
        self.map_subscriber = rospy.Subscriber(f'{self.robot_name}/map', OccupancyGrid, self.verify_testcase)

        # keep the process going
        self.rate = rospy.Rate(10)
        rospy.spin()

    def verify_testcase(self, msg):
        # write some test case methods
        assert msg is not None
        self.map = np.array(msg.data)
        rospy.loginfo(f'{self.map}')


if __name__ == "__main__":
    TA = TestAMCL()
