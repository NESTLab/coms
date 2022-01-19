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

# Obtain ROS environment variables
source /opt/ros/noetic/setup.bash
# Install ROS Packages
rosdep install -i --from-path /root/catkin_ws/src --rosdistro $ROS_DISTRO -y
# Enforce C++ language standard
catkin config --cmake-args -DCMAKE_CXX_STANDARD=20 -DPYTHON_EXECUTABLE=/usr/bin/python3

exec supervisord -c /app/supervisord.conf