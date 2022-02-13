SHELL=bash
WORKDIR=/root/catkin_ws/src
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
COVERALLS_REPO_TOKEN=Vqz48pxOpQ8k0D7yp4tQs3LUVTaSEqIcj

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

