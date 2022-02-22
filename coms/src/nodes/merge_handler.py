#!/usr/bin/env python3
import rospy
from coms.srv import MergeMap, MergeMapResponse, MergeMapRequest, TriggerMerge, TriggerMergeResponse, TriggerMergeRequest
from nav_msgs.srv import GetMap, GetMapResponse, GetMapRequest
from nav_msgs.msg import OccupancyGrid
from mapmerge.keypoint_merge import orb_mapmerge
from mapmerge.ros_utils import pgm_to_numpy, numpy_to_ros, ros_to_numpy
import numpy as np
import rosbag


class MergeHandler:
    def __init__(self):
        # load rospy parameters
        self.robot_name = rospy.get_param('ns', 'robot_1')
        self.starting_map_path = rospy.get_param('starting_map', '')
        self.logging = bool(rospy.get_param('logging', 0))
        self.debug = bool(rospy.get_param('debug', 1))

        # init ros
        rospy.init_node("mergeHandler", anonymous=True)
        self.merge_service = rospy.Service("merge", MergeMap, self.merge_cb) # depreciated
        self.get_map_service = rospy.Service("get_map", GetMap, self.serve_map)
        self.trigger_merge = rospy.Service("trigger_merge", TriggerMerge, self.trigger_merge_cb)
        self.map_publisher = rospy.Publisher("merged_map", OccupancyGrid, queue_size=10)
        self.gmapping_subscriber = rospy.Subscriber("map", OccupancyGrid, self.gmapping_cb)
        self.rate = rospy.Rate(10)  # 10hz

        # logging
        if self.logging:
            self.bag = rosbag.Bag(f'{self.robot_name}_log.bag', 'w')

        # map data
        self.seq = 0
        self.latest_map = np.array([])
        self.inital_shape = (0, 0)
        if self.starting_map_path != "":
            self.latest_map = pgm_to_numpy(self.starting_map_path)
            self.inital_shape = self.latest_map.shape
            print(f"starting with inital map of shape {self.inital_shape}")

    def run(self) -> None:
        rospy.loginfo(f'{self.robot_name} merge handler node starting')
        while not rospy.is_shutdown():
            if self.latest_map.any():
                self.publish_latest()
            self.rate.sleep()
        rospy.loginfo(f'{self.robot_name} merge handler node shutting down')
        if self.logging:
            self.bag.close()

    def publish_latest(self) -> None:
        occ = self.create_occupancy_msg(self.seq)
        self.seq += 1
        self.map_publisher.publish(occ)

    def gmapping_cb(self, msg: OccupancyGrid) -> None:
        """
        callback for gmapping,
        TODO gmapping:
            increase map update speed
            set map transform to remain the same after merges.
        """

        if self.logging:
            self.bag.write('map', msg)


        breakpoint()
        MergeHandler.parse_and_save(msg, self.latest_map)
        # unpack gmapping
        new_map = ros_to_numpy(msg.data).reshape(-1,msg.info.width)
        self.merge_map(new_map)

    @staticmethod
    def parse_and_save(msg, old_map):
        """
        takes in an occupancy grid and saves it as an npy file
        """
        new_map = ros_to_numpy(msg.data).reshape(-1, msg.info.width)
        with open(f'map_{msg.header.seq}.npy', 'wb') as f:
            np.save(f, new_map)
        with open(f'old_map{msg.header.seq}.npy', 'wb') as f:
            np.save(f, old_map)

    # depreciated
    def merge_cb(self, req: MergeMapRequest) -> MergeMapResponse:
        new_map = ros_to_numpy(req.map.data).reshape(-1, req.map.info.width)
        return self.merge_map(new_map)

    def trigger_merge_cb(self, req: TriggerMergeRequest) -> TriggerMergeResponse:
        """
        Gonna just do a ros merge for now:
        """
        name = req.robot_id
        new_map = self.get_map(name)
        return self.merge_map(new_map)

    def merge_map(self, new_map: np.array([])) -> bool:
        """
        merges a map.
        configurable number of retries and error catching
        """

        if not self.latest_map.any():
            self.latest_map = new_map
            return True

        try:
            print(new_map.shape)
            print(self.latest_map.shape)
            merged = orb_mapmerge(new_map, self.latest_map)
            # TODO checks and maybe a lock? depends on the rospy callback threading
            self.latest_map = merged
            return True
        except Exception as e:
            rospy.logerr(f"Could not merge maps: {e}")
        return False

    def get_map(self, name: str) -> np.array([]):
        """
        gets the latest map from map service
        """
        if name[0] != '/':
            name = '/' + name

        rospy.wait_for_service(name + '/get_map')
        while 1:
            try:
                map_service = rospy.ServiceProxy(name + '/get_map', GetMap)
                resp = map_service()
                return ros_to_numpy(resp.map.data).reshape(-1, resp.map.info.width)
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" % e)
            self.rate.sleep()

    def serve_map(self, _) -> GetMapResponse:
        """
        serves the latest map
        """
        occ = self.create_occupancy_msg(self.seq)
        return GetMapResponse(occ)

    def create_occupancy_msg(self, seq: int) -> OccupancyGrid:
        occ = OccupancyGrid()
        # header
        occ.header.seq = seq
        # metadata
        occ.info.height = self.latest_map.shape[0]
        occ.info.width = self.latest_map.shape[1]
        # data
        occ.data = tuple(numpy_to_ros(self.latest_map.flatten()))
        return occ


if __name__ == "__main__":
    MH = MergeHandler()
    MH.run()
