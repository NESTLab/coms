#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from multiprocessing import Process, Array
from nav_msgs.srv import GetMap, GetMapResponse


class MapMerger:
    """
    This class handles providing the latest local map for the rest of the ros system to access
    general operations:
        continuously call ros gmapping for the latest local map
        merge gmapping map onto merged map for continuity
            * save transform from previous local merges *
            * data averaging to be robust to noise *
        continuously publish latest merged map on amcl / move base topic
        merge forgien maps upon avaiability
    """

    def __init__(self):
        # GET PARAMS
        self.robot_name = rospy.get_param('~robot_name', 'tb0')
        self.map_service = rospy.get_param('~map_service', 'dynamic_map')

        # DATA
        self.latest_map = np.array([])

        # INIT ROS
        rospy.init_node(f"{self.robot_name}_MapMerger")
        self.map_publisher = rospy.Publisher(f'{self.robot_name}/map', OccupancyGrid, queue_size=10)

        # keep the process going
        self.rate = rospy.Rate(10)  # 10hz

    def run(self):
        """
        main running loop
        """
        rospy.loginfo(f'{self.robot_name} map_merger node starting')

        while not rospy.is_shutdown():
            mapdata = self.get_latest()
            # add some merge jumbo here
            self.publish_latest(mapdata)
            self.rate.sleep()

        rospy.loginfo(f'{self.robot_name} map_merger node shutting down')

    def publish_latest(self, mapdata):
        """
        publishes the latest map
        """
        self.map_publisher.publish(mapdata)

    def get_latest(self):
        """
        gets the latest map from map service
        """
        rospy.wait_for_service(self.map_service)
        while 1:
            try:
                map_service = rospy.ServiceProxy(self.map_service, GetMap)
                response = map_service()
                return response.map
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" % e)


if __name__ == '__main__':
    mm = MapMerger()
    mm.run()
