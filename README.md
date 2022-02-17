[![Build Status](https://app.travis-ci.com/tylerferrara/coms.svg?branch=main)](https://app.travis-ci.com/tylerferrara/coms)
[![Coverage Status](https://coveralls.io/repos/github/tylerferrara/coms/badge.svg?branch=main)](https://coveralls.io/github/tylerferrara/coms?branch=main)

# Coms 📡
Distributed Communication Protocol for Robots running [ROS1](https://www.ros.org).

## Overview 🔍
Coms acts as a middleman between all network traffic, regardless of the deployment environment. It takes care of all robot-to-robot communications, whether they are deployed on raspberry pis, or run within a simulation environment such as Gazebo. This package enables your ROS nodes to rely on a standard communication protocol for external message passing. In essence, Coms works as a ROS1 implementation of a multi-master approach for orchestrating distributed ad-hoc networks.

## ROS Topics 💬
For simulations, the Sim._broadcaster sends a `nearby.msg` to the `/nearby_robots` topic whenever it sees another robot in line of site. Different physics simulations, such as ARGos and Gazebo, have their own way of determining line-of-site. Based on the configuration run by
the `ros-net-sim` package, a given physics_sim may be invoked to match the robot's environment.

ARGos3 - Line-of-site topics are generated for each robot run within its physics engine.
The broadcaster subscribes to a given `/robot<num>/line-of-site` topic and re-broadcasts them on a regular interval to the `/nearby_robots` topic for other ROS nodes.

Gazebo - TODO

## Message Handling ❌
Each robot running a Coms node is required to choose a __unique static IP__ that will not conflict with other robots. When each robot is within proximity with each other, they attempt to establish a connection. Network conditions vary depending on the deployment environment, which can result in abrupt disconnection or packet loss. For this reason, each message may define the number of retires, timeout, and error handler to manage failures.

## Environments 🌏
Coms needs to know where it's deployed so it can forward messages correctly. For now, there are only two environments to choose from: Raspberry Pi & Simulation

### Raspberry Pi
This assumes the ROS node is running on hardware that resembles a __wifi-enabled__ Raspberry Pi. The network card will then be programmed into the ad-hoc mode, and attempt to send message payloads over WiFi as TCP/UDP network packets.

### Simulation | __Ubuntu Only__
This assumes all nodes are running on the same host machine. In this case, there must be __no collisions between name-spacing__ of robots. Coms sets up a virtual network simulator on the host machine, where each robot is assigned a virtual network device for all communications. For this reason, this environment is __only supported by Ubuntu 18.04+__, as this feature relies on the OS to make network tunnels.

Coms also deploys a physics simulator that analyses the environment of the robots to control network reliability. Currently, this is implemented with a rudimentary form of collision detection and proximity. However, feel free to read the docs of the project this was forked from, [ROSNetSim](https://arxiv.org/pdf/2101.10113.pdf), for guidance in implementing more sophisticated raytracing.

## Usage 🛠

### For Simulation Environments:


It's recommended to attach a Coms node to every Robot that wishes to communicate with the network. In ROS, this means adding it to your launch file.
```launch
<launch>

    <arg name="my_robot"/>

    <node name="network_coms" pkg="coms">
        <arg name="environment" value="[ pi | sim ]"/>
        <arg name="ip" value="192.168.0.5"/>
    </node>

<launch>
```
Alternatively, you can manually run Coms as a script which
```zsh
rosrun coms main.py --environment=[ pi | sim ] --ip=192.168.0.5
```
or start it using it's launch file:
```zsh
roslaunch coms main.launch --environment=[ pi | sim ] --ip=192.168.0.5
```

## Create a Message 📨
> Simply create a class, within `/msg`, which implements the _Message interface_.
> Here's an example of what this looks like:

To create a new Message we specify the data we want to pass back and forth and store them as class attributes. For this example, a `Meeting` message holds `location`, `time`, and `ack` information. At the same time, we define the number of `retries` to keep sending the same message and define when a message becomes stale - if it hasn't received a response in `timeout` seconds. These values override the defaults within the Message interface, so they are not explicitly required.

```py
class Meeting(Message):

 retries:int = 5
 timeout:int = 4
 
 def __init__(self, location:Tuple[int] = None, time:time.time = None, ack:bool = None) -> None:
 self.location = location
 self.time = time
 self.ack = ack
 
 ...

```

Next, we implement the `produce_payload` class function to handle the packaging of our message information. There are no restrictions on how you may produce this payload, as long as the end result are bytes. Additionally, we implement `consume_payload` for deconstructing a payload into a new message object. Ensure all attributes are populated within this new message object so they can be handled when received.

```py
class Meeting(Message):

 retries:int = 5
 timeout:int = 4

 def __init__(self, location:Tuple[int] = None, time:time.time = None, ack:bool = None) -> None:
 self.location = location
 self.time = time
 self.ack = ack
 
 @classmethod
 def produce_payload(location:Tuple[int], time:time.time, ack:bool = False) -> bytes:
 payload:bytes = b''
 # TODO: Package the arguments into bytes
 return payload

 @classmethod
 def consume_payload(payload:bytes) -> Meeting:
 msg = Meeting()
 # TODO: Convert the payload into a Meeting object
 return msg

 ...

```

Optionally, messages can implement an error handling strategy for when a Message exhausts all retries and cannot be sent over the network.

```py
class Meeting(Message):

 retries:int = 5
 timeout:int = 4

 def __init__(self, location:Tuple[int] = None, time:time.time = None, ack:bool = None) -> None:
 self.location = location
 self.time = time
 self.ack = ack
 
 @classmethod
 def produce_payload(location:Tuple[int], time:time.time, ack:bool = False) -> bytes:
 payload:bytes = b''
 # TODO: Package the arguments into bytes
 return payload

 @classmethod
 def consume_payload(payload:bytes) -> Meeting:
 msg = Meeting()
 # TODO: Convert the payload into a Meeting object
 return msg

 @classmethod
 def on_failure() -> None:
 # TODO: Optionally handle errors
```


