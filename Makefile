SHELL=bash
WORKDIR=/root/catkin_ws/src
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash

test:
	$(SETUP); \
	python3 -m unittest discover $(WORKDIR)/coms/tests

test-ping:
	$(SETUP); \
	python3 -m unittest $(WORKDIR)/coms/tests/test_ping.py

coverage:
	coverage run --source=coms -m unittest discover $(WORKDIR)/coms/tests/; \
	coveralls

check-health:
	source $(WORKDIR)/health-check.sh

