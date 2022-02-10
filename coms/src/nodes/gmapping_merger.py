#!/usr/bin/env python3
from merger import ROSMerger
from std_msgs.msg import Header
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid, MapMetaData
from coms.srv import MergeMap
import numpy as np
import rospy


class GmappingMerger(ROSMerger):

    def __init__(self) -> None:
        super().__init__()
        self.robot_name = rospy.get_param('robot_name', 'tb0')
        self.request_service = f'{self.robot_name}/merge'
        self.map_service = rospy.get_param('map_service', 'dynamic_map')

        self.latest_map = np.array([])

        rospy.init_node(f"{self.robot_name}_GmappingMerger")
        self.rate = rospy.Rate(10)  # 10hz

    def run(self) -> None:
        rospy.loginfo(f'{self.robot_name} gmapping merger node starting')

        while not rospy.is_shutdown():
            resp = self.get_latest()
            if self.check_new(resp):
                self.request_merge(resp)
            self.rate.sleep()

        rospy.loginfo(f'{self.robot_name} gmapping merger node shutting down')

    def request_merge(self, new_map: OccupancyGrid) -> bool:
        """
        Requests a merge from the specified ros service {request_service}.
        """
        rospy.wait_for_service(self.request_service)
        attempts = 0
        max_attemps = 10
        try:
            req = new_map.map
            request_service = rospy.ServiceProxy(self.request_service, MergeMap)
            return request_service(req)
        except rospy.ServiceException as e:
            attempts += 1
            rospy.loginfo("service call failed: %s" % e)
            if attempts >= max_attemps:
                return False
        return False

    def get_latest(self):
        """
        gets the latest map from map service
        """
        rospy.wait_for_service(self.map_service)
        while 1:
            try:
                map_service = rospy.ServiceProxy(self.map_service, GetMap)
                return map_service()
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" % e)
            self.rate.sleep()

    def check_new(self, new_map: OccupancyGrid) -> bool:
        if not self.latest_map.any():
            # if we don't have a latest map
            return True

        temp = np.array(new_map.data)
        # return True if maps don't match, continue to merge req
        return not np.all(self.latest_map, temp)


if __name__ == '__main__':
    GM = GmappingMerger()
    GM.run()
