import rospy
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from mapmerge.ros_utils import ros_to_numpy
import sys
import numpy as np


def map_debugger(name):
    if name == '' or not name:
        return
    map_service = name + "/get_map"
    print(f'Trying map service [{map_service}]')

    rospy.init_node("debugger", anonymous=True)
    rate = rospy.Rate(10)

    rospy.wait_for_service(map_service)
    while 1:
        try:
            map_service = rospy.ServiceProxy(map_service, GetMap)
            resp = map_service()
            map = ros_to_numpy(resp.map.data)
            print(resp.map.header)
            print(resp.map.info)
            parse_and_save(resp.map)
            # TODO extract map size info
            plt.imshow(map.reshape(-1, resp.map.info.width))
            plt.show()
            return
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" % e)
        rate.sleep()


def parse_and_save(msg):
    """
    takes in an occupancy grid and saves it as an npy file
    """
    new_map = ros_to_numpy(msg.data).reshape(-1, msg.info.width)
    with open(f'map_{msg.header.seq}.npy', 'wb') as f:
        np.save(f, new_map)


def map_sub_debugger(name):
    if name == '' or not name:
        return
    map_service = name + "/merged_map"
    print(f'Trying map service [{map_service}]')

    rospy.init_node("debugger", anonymous=True)
    rate = rospy.Rate(10)
    sub = rospy.Subscriber(map_service, OccupancyGrid, parse_and_save)
    rospy.spin()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: mapDebugger.py <robot name>")
    map_debugger(sys.argv[1])
    # map_sub_debugger(sys.argv[1])
