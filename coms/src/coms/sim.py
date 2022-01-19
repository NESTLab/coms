from subprocess import Popen, check_output, DEVNULL, call
import time
from typing import List


# Returns PID of the launch process
def launch_sim_network() -> Popen:
    ros_setup_script = "/opt/ros/noetic/setup.sh"
    devel_setup_script = "/root/catkin_ws/devel/setup.sh"
    command = "roslaunch example gazebo.launch"
    # This launch file explicitly creates 3 network devices
    num_devices = 3
    cmd = "sh {0} && sh {1} && {2}".format(ros_setup_script, devel_setup_script, command)
    proc = Popen(cmd, stdout=DEVNULL, stderr=DEVNULL, shell=True)
    # Wait for the OS to create all network devices
    while len(get_device_numbers()) != num_devices:
        time.sleep(0.1)
    return proc


def terminate_sim_network(proc: Popen) -> None:
    while len(get_device_numbers()) != 0:
        devices = get_device_numbers()
        print(devices)
        proc.kill()
        for dev in devices:
            remove_net_tunnel(dev)
        time.sleep(0.1)
    remove_net_rules()


def get_device_numbers() -> List:
    devices = []
    out = check_output(["sudo", "-S", "ip", "tuntap", "list", "dev", "mode", "tun"])
    for line in out.splitlines():
        parts = line.decode().split(':')
        dev_number = int(parts[0][3], 10)
        devices.append(dev_number)
    return devices


def remove_net_tunnel(i: int) -> None:
    call(["sudo", "-S", "ip", "tuntap", "del", "dev", "tun" + str(i), "mode", "tun"])
    call(["sudo", "ip", "link", "delete", "dev", "tun" + str(i)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 1)])
    call(["sudo", "ip", "rule", "del", "table", str(i + 101)])


def remove_net_rules() -> None:
    call(["sudo", "ip", "rule", "add", "pref", "0", "from", "all", "lookup", "local"])
    call(["sudo", "ip", "rule", "del", "pref", "10", "from", "all", "lookup", "local"])


# If tunnel devices are NOT present
# - Launch -> example gazebo.launch

# Create SIMPLEST form of:
# listener and broadcaster

# Simply send and recieve strings

# Create a Message Interface

# Create a dummyMessage to impliment it (sends a string as a payload)
