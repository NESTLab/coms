SHELL=bash
SETUP=source /opt/ros/noetic/setup.bash;source /root/catkin_ws/devel/setup.bash
COVERALLS_REPO_TOKEN=Vqz48pxOpQ8k0D7yp4tQs3LUVTaSEqIcj
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

install-coms:
	install-deps; \
	pip install -e $(WORKDIR)/coms

argos:
	$(SETUP); \
	argos3 -c $(WORKDIR)/argos_bridge/argos_worlds/multi_robots.argos

