#!/usr/bin/env python3
from merger import ROSMerger
from std_msgs.msg import Header
from nav_msgs.srv import GetMap, GetMapResponse
from nav_msgs.msg import OccupancyGrid, MapMetaData
from coms.srv import MergeMap, TriggerMerge, TriggerMergeResponse, TriggerMergeRequest
import numpy as np
import rospy


class NetSimMerger(ROSMerger):

    def __init__(self) -> None:
        super().__init__()
        self.robot_name = rospy.get_param('robot_name', 'tb0')
        self.merge_service = f'{self.robot_name}/merge'
        self.map_service = f'{self.robot_name}/get_map'

        self.netsim_merge = rospy.Service(self.map_service, TriggerMerge, self.get_netsim_map)

        rospy.init_node(f"{self.robot_name}_NetSimMerger")
        self.rate = rospy.Rate(10)  # 10hz
        rospy.spin()

    def request_merge(self, new_map: OccupancyGrid) -> bool:
        """
        Requests a merge from the specified ros service {merge_service}.
        """
        rospy.wait_for_service(self.merge_service)
        attempts = 0
        max_attemps = 10
        try:
            req = new_map.map
            merge_service = rospy.ServiceProxy(self.merge_service, MergeMap)
            return merge_service(req)
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

    def get_netsim_map(self, req: TriggerMergeRequest):
        """
        use network simulation to trade maps with another robot
        """
        pass


if __name__ == '__main__':
    NSM = NetSimMerger()
