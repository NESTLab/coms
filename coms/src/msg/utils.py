from coms.constants import ENCODING
from coms.srv import MergeMap
from nav_msgs.msg import OccupancyGrid
from ros_utils import ros_to_numpy, numpy_to_ros
from nav_msgs.srv import GetMap
import rospy


MESSAGE_REGISTRY = {
    0: 'Merge',
    1: 'Ping',
}


def extract_payload_id(payload: bytes) -> int:
    s = payload.decode(ENCODING)
    if len(s) == 0:
        return -1
    return int(s[0], 10)


def get_message_type(payload: bytes) -> str or None:
    id = extract_payload_id(payload)
    if id not in MESSAGE_REGISTRY:
        return None
    return MESSAGE_REGISTRY[id]

# TODO: MUST TEST
def request_merge(ros: rospy, foriegn_map: OccupancyGrid) -> None:
    """
    Requests a merge from the specified ros service {merge_service}.
    """
    robot_name = ros.get_name()
    ros.wait_for_service(f'{robot_name}/merge')
    attempts = 0
    max_attemps = 10
    try:
        req = foriegn_map
        merge_service = ros.ServiceProxy(f'{robot_name}/merge', MergeMap)
        return merge_service(req)
    except ros.ServiceException as e:
        attempts += 1
        ros.loginfo("service call failed: %s" % e)
        if attempts >= max_attemps:
            return False
    return False

# TODO: MUST TEST
def get_latest(ros: rospy) -> None:
    """
    gets the latest map from map service
    """
    robot_name = ros.get_name()

    ros.wait_for_service(f"{robot_name}/get_map")
    while 1:
        try:
            map_service = ros.ServiceProxy(f"{robot_name}/get_map", GetMap)
            return ros_to_numpy(map_service().map)
        except ros.ServiceException as e:
            ros.loginfo("service call failed: %s" % e)
        ros.Rate(10).sleep()
    return None

def create_occupancy_msg(map: np.ndarray, seq: int = 0) -> OccupancyGrid:
    occ = OccupancyGrid()
    occ.header.seq = seq
    occ.data = tuple(numpy_to_ros(map.flatten()))
    return occ
