#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import ctypes
from multiprocessing import Process, Array, Lock
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
        self.map_size = rospy.get_param('~map_size', 100)

        # DATA
        self.seq = 0
        # map lock handles preventing multiple processes from editing map at once
        self.map_lock = Lock()
        # edit lock prevents processes from starting a merge before the other: gmapping vs dag merge
        self.edit_lock = Lock()
        # shared memory for numpy array map. can be edited (merged) by two different threads looking on two networks
        self.shared_map = MapMerger.to_numpy(Array(ctypes.c_int8, self.map_size ** 2, lock=self.map_lock))

        # PROCESSES
        self.gmapping_merger = Process(target=self.get_latest_gmapping(), args=(self,))
        self.netsim_merger = Process(target=self.get_latest_netsim(), args=(self,))

        # INIT ROS
        rospy.init_node(f"{self.robot_name}_MapMerger")
        self.map_publisher = rospy.Publisher(f'{self.robot_name}/map', OccupancyGrid, queue_size=10)

        # keep the process going
        self.rate = rospy.Rate(10)  # 10hz

    def run(self):
        """
        main running loop
        """
        self.gmapping_merger.start()
        self.netsim_merger.start()
        rospy.loginfo(f'{self.robot_name} map_merger node starting')

        while not rospy.is_shutdown():
            self.publish_latest()
            self.rate.sleep()

        rospy.loginfo(f'{self.robot_name} map_merger node shutting down')
        self.gmapping_merger.join()
        self.netsim_merger.join()

    def publish_latest(self):
        """
        publishes the latest map
        """
        map_message = OccupancyGrid()
        map_message.header = self.seq
        self.map_lock.acquire()
        map_message.data = self.shared_map
        self.map_lock.release()
        self.map_publisher.publish()

        self.seq += 1

    def merge_into_shared(self, new_map):
        """
        merges a new map into the shared memory map
        """

        self.edit_lock.acquire()

        # perform map merge
        merged_map = new_map

        # save new map in shared memory
        self.map_lock.acquire()
        self.shared_map = merged_map
        self.map_lock.release()

        self.edit_lock.release()

    def get_latest_gmapping(self):
        """
        gets the latest map from map service
        """
        rospy.wait_for_service(self.map_service)
        while 1:
            try:
                map_service = rospy.ServiceProxy(self.map_service, GetMap)
                response = map_service()
                latest_map = np.array(response.map.data)
                #self.merge_into_shared(latest_map)
            except rospy.ServiceException as e:
                rospy.loginfo("service call failed: %s" % e)

    def get_latest_netsim(self):
        """
        gets the latest map from local robots
        """
        pass

    @staticmethod
    def to_numpy(array):
        return np.frombuffer(array.get_obj())


if __name__ == '__main__':
    mm = MapMerger()
    mm.run()
