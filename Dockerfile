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
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'; \
    wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -; \
    apt-get update; \
    apt-get install -y gazebo11 libgazebo11-dev; \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc; \
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc;

# Build ARGos3 from source
RUN apt-get install -y cmake libfreeimage-dev libfreeimageplus-dev \
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
  make install;

# Install project dependencies
RUN rosdep init; \
  rosdep update; \
  echo "rosdep install --from-paths /root/catkin_ws/src --rosdistro noetic -y" >> ~/.bashrc; \
  echo "cd /root/catkin_ws" >> ~/.bashrc;

EXPOSE 8080

CMD ["/app/entrypoint.sh"]