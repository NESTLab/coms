import rospy
import matplotlib.pyplot as plt
from nav_msgs.srv import GetMap
from mapmerge.ros_utils import ros_to_numpy
import sys


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

            # TODO extract map size info
            plt.imshow(map.reshape(-1,resp.map.info.width))
            plt.show()
            return
        except rospy.ServiceException as e:
            rospy.loginfo("service call failed: %s" % e)
        rate.sleep()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("usage: mapDebugger.py <robot name>")
    map_debugger(sys.argv[1])
