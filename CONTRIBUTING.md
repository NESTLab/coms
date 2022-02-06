# Setup
> It's highly reccomended you develop in a dockerized environment. Umong many advantages, this provides a static dev environment complete with all the necisary dependencies.

1. Follow the instructions to download [Docker](https://www.docker.com/products/docker-desktop) and [Docker-Compose](https://docs.docker.com/compose/install/) on your machine.
2. Build the image. This will take a while when building the first time, but is cached for future builds.
```zsh
$ docker build .
```
3. Start the dev environment. This sets up the networking and startup scripts for development.
```zsh
$ docker-compose up
```
4. Start a bash instance within the container. You can either specify the container by name, or use this script to grab the ID of the running container. Therefore, if you're also running other containers, make sure you execute the bash shell in the correct container.
```zsh
$ docker exec -it $(docker ps --format "{{.ID}}") /bin/bash
```

# Testing
All tests are currently invoked using the python `unittest` library. To run all tests, you can use the `discover` command and specify the coms/tests directory:
```zsh
# Run ALL tests in container
$ python3 -m unittest discover /root/catkin_ws/src/coms/tests
```
The above command will run every test found in `/root/catkin_ws/src/coms/tests`.


