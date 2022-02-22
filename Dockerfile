FROM ubuntu:focal

# Setup environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1024 \
    DISPLAY_HEIGHT=768 \
    RUN_XTERM=yes \
    RUN_FLUXBOX=yes

# Install git, supervisor, VNC, & X11 packages
RUN set -ex; \
    apt-get update; \
    apt-get install -y \
      sudo \
      lsb-core \
      gnupg2 \
      curl \
      bash \
      fluxbox \
      git \
      net-tools \
      novnc \
      supervisor \
      x11vnc \
      xterm \
      xvfb \
      python3-pip; \
    git clone https://github.com/theasp/docker-novnc.git /app;

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'; \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -; \
    apt-get update; \
    apt-get install ros-noetic-desktop-full -y; \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc; \
    apt-get install -y python3-rosdep python3-rosinstall ros-noetic-turtlebot3 ros-noetic-dwa-local-planner ros-noetic-gmapping ros-noetic-rviz python3-rosinstall-generator python3-wstool build-essential python3-rosdep;

# Install Gazebo
RUN wget -O /tmp/gazebo5_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo5_install.sh; sudo sh /tmp/gazebo5_install.sh; \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc;

# Build ARGos3 from source
RUN apt-get install -y libgsl-dev cmake libfreeimage-dev libfreeimageplus-dev \
  qt5-default freeglut3-dev libxi-dev libxmu-dev liblua5.3-dev \
  lua5.3 doxygen graphviz libgraphviz-dev asciidoc; \
  git clone https://github.com/ilpincy/argos3.git; \
  cd argos3; \
  mkdir build_simulator; \
  cd build_simulator; \
  cmake ../src; \
  make; \
  make doc; \
  echo '/usr/local/lib' >> /etc/ld.so.conf; \
  echo "sudo ldconfig" >> ~/.bashrc; \
  make install

# Install kheperaiv robot to ARGos3
RUN git clone https://github.com/ilpincy/argos3-kheperaiv.git; \
  cd argos3-kheperaiv && mkdir build_sim && cd build_sim; \
  cmake -DCMAKE_BUILD_TYPE=Release ../src; \
  make; \
  sudo make install

# Install extra dependencies
RUN sudo apt-get update; \
  sudo apt-get upgrade -y; \
  sudo apt-get install -y \
    python-is-python3 \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    iproute2 \
    ros-noetic-catkin \
    python3-catkin-tools \
    libtf2-ros-dev

# Add ROS dependent scripts
RUN rosdep init; \
  rosdep update; \
  mkdir -p /root/catkin_ws/src; \
  echo "cd /root/catkin_ws;" >> ~/.bashrc; \
  echo "source /root/catkin_ws/devel/setup.bash;" >> ~/.bashrc; \
  echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/opt/ros/noetic/lib" >> ~/.bashrc; \
  echo "export ARGOS_PLUGIN_PATH=$HOME/catkin_ws/src/argos_bridge/ros_lib_links" >> ~/.bashrc; \
  echo "export ARGOS_PLUGIN_PATH=$ARGOS_PLUGIN_PATH:$HOME/catkin_ws/devel/lib" >> ~/.bashrc; \
  echo "export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1" >> ~/.bashrc
EXPOSE 8080

CMD ["/root/catkin_ws/src/entrypoint.sh"]