SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
COVERALLS_REPO_TOKEN=x914kIwyMzM7CLf2ixlx0xwW84IF8tUPT
PATH_TO_MAKEFILE=$(abspath $(lastword $(MAKEFILE_LIST)))
WORKDIR=$(shell dirname $(PATH_TO_MAKEFILE))

test:
	$(SETUP); \
	python3 -m unittest discover $(WORKDIR)/coms/tests

test-ping:
	$(SETUP); \
	python3 -m unittest $(WORKDIR)/coms/tests/test_ping.py

test-sim:
	$(SETUP); \
	python3 -m unittest $(WORKDIR)/coms/tests/test_sim.py

coverage:
	$(SETUP); \
	export COVERALLS_REPO_TOKEN=$(COVERALLS_REPO_TOKEN); \
	cd $(WORKDIR); \
	coverage run --source=coms -m unittest discover $(WORKDIR)/coms/tests/; \
	coveralls

check-health:
	source $(WORKDIR)/health-check.sh

install-deps:
	pip install -r $(WORKDIR)/coms/requirements-dev.txt; \
	pip install -r $(WORKDIR)/coms/requirements.txt

install-coms: install-deps; \
	pip install -e $(WORKDIR)/coms

install-argos-net-sim:
	pip install -e $(WORKDIR)/ros-net-sim/network_coordinator

install: install-coms install-argos-net-sim

argos-demo:
	$(SETUP); \
	argos3 -c $(WORKDIR)/argos_bridge/argos_worlds/multi_robot_dan_maze.argos

gazebo-demo:
	$(SETUP); \
	roslaunch turtlebot3_gazebo multi_turtlebot3_all.launch

gazebo-net-sim:
	$(SETUP); \
	roslaunch example gazebo.launch

argos-net-sim:
	$(SETUP); \
	roslaunch example argos.launch

coms-net:
	$(SETUP); \
	roslaunch coms net.launch
