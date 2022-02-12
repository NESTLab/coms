#!/bin/bash
set -ex

RUN_FLUXBOX=${RUN_FLUXBOX:-yes}
RUN_XTERM=${RUN_XTERM:-yes}

case $RUN_FLUXBOX in
  false|no|n|0)
    rm -f /app/conf.d/fluxbox.conf
    ;;
esac

case $RUN_XTERM in
  false|no|n|0)
    rm -f /app/conf.d/xterm.conf
    ;;
esac

# Install dev packages
pip install -r /root/catkin_ws/src/coms/requirements-dev.txt
pip install -r /root/catkin_ws/src/coms/requirements.txt
# Install local packages
pip install -e /root/catkin_ws/src/coms
# Obtain ROS environment variables
source /opt/ros/noetic/setup.bash
cd /root/catkin_ws
# Install ROS Packages
rosdep install -i --from-path /root/catkin_ws/src --rosdistro $ROS_DISTRO -y
# Enforce C++ language standard
catkin config --cmake-args -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3
# Make the project
catkin_make --cmake-args -DCMAKE_CXX_STANDARD=17 -DPYTHON_EXECUTABLE=/usr/bin/python3

exec supervisord -c /app/supervisord.conf