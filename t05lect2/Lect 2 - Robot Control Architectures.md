# Lect 2 - Robot Control Architectures

## Definition

- Software framework

## Features

- parallelism
- runtime flexibility
- modularity
- robustness
- ease of use
- performance

## Classification

- Deliberative paradigm: static environment; reliable task execution; no reaction against external disturbances; need to replan
- Reactive paradigm: No task planning;
- Hybrid paradigm: Task planning to achieve global objectives; plan execution monitoring; reacts against external stimuli

## ROS

- Open source, meata opearting system for robots
- Hardware abstraction, device control, common functionality, process communication, package management
- Standfor, Willow Garage Institute
- ROS Paper 2009:

	- Design goals: Peer to peer, tools based, multi lingual, thin, open source.
	- Nomenclature: Nodes, messages, topic, service
	- Use cases: debug a single node; logging and playback; packaged subsystems; collaborative development (Source Forge); virtualization and monitoring; composition of functionality; transformations;

- RosCon, ROS-I Industrial Consortium, ROS2
- C (2010) - Noetic Ninjemys (2020)
- ROS Ingredients: Nodes, topics, packages, Master, peer-to-peer, services, parameters
- ROS2: comm based on DDS; QoS; no Master;
- Topics: Publisher, subscriber, message
- Services: Request, Response, Client, Service, Server
- OpenMora architecture (central DB)

## ROS2 Commands

### Topics

- ros2 run <package> <executable>
- ros2 node info <node>
	- Subscribers
	- Publishers
	- Service servers
	- Service clients
	- Actions servers/clients (non blocking)
- ros2 topic list [-t]
- ros2 topic info <topic-name>
- ros2 interface show <topic-name>!!! TOPIC-TYPE!!!
- ros2 topic pub /<topic-name> <type> <args> [--rate f] 
- ros2 topic echo <topic-name>
- ros2 run rqt_graph 
- rqt_graph

```
ros2 run turtlesim turtlesim_node
ros2 node list
ros2 node info /turtlesim
ros2 topic list -t
ros2 topic info /turtle1/cmd_vel

ros2 interface show turtlesim/msg/Color
ros2 topic pub /turtle1/color_sensor turtlesim/msg/Color '{r: 25, g: 25, b: 0}' -1

ros2 interface show turtlesim/msg/Pose
ros2 topic pub /turtle1/pose turtlesim/msg/Pose '{x: 1, y: 0, b: 0}' -1

ros2 interface show geometry_msgs/msg/Twist
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1, y: 0.1, z: 0}, angular: {x: 0, y: 0, z: 1}}' --rate 5

ros2 topic echo /turtle1/cmd_vel
ros2 topic echo /turtle1/pose

ros2 topic -h 

ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot
```

### Services

- ros2 run rqt_plot rqt_plot
- ros2 service list -t
- ros2 service type <service_name>
- ros2 service find <service_type>
- ros2 interface show <service_type>
- ros2 service call <service_name> <service_type> <arguments>

```
ros2 service list -t
ros2 service type /turtle1/set_pen
ros2 service find turtlesim/srv/SetPen
ros2 interface show turtlesim/srv/SetPen
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen '{r: 255, g: 100, b: 100}'

ros2 service -h

ros2 service list -t
ros2 service type /kill
ros2 service find turtlesim/srv/Kill
ros2 interface show turtlesim/srv/Kill

ros2 service find std_srvs/srv/Empty
ros2 interface show std_srvs/srv/Empty

ros2 service find turtlesim/srv/Spawn
ros2 interface show turtlesim/srv/Spawn

ros2 service call /kill turtlesim/srv/Kill "name: 'turtle1'"
ros2 service call /clear std_srvs/srv/Empty

ros2 interface show turtlesim/srv/Spawn
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1.0, y: 1.0, theta: 10.0, name: 'turtle4'}"


``` 

### Params

- ros2 param list
- ros2 param get <node_name> <parameter>
- ros2 param set <node_name> <parameter> <new_value>
- ros2 param dump <node_name> > <file_name>
- ros2 param load <node_name> <file_name>

```
ros2 param list

ros2 node list
ros2 param get /turtlesim background_b
ros2 param set /turtlesim background_b 128
ros2 param dump /turtlesim > borrame.txt
nano borrame.txt
ros2 param load /turtlesim borrame.txt
```

### Bags

- ros2 bag record <topic-names> [-a para todos los topics] [-d segundos] [-o fichero salida]
- ros2 bag info <filename>
- ros2 bag play <filename>

```
ros2 bag record -a -d 5 -o mybag2.bag
ros2 bag info mybag2.bag

ros2 bag record /turtle1/cmd_vel geometry_msgs/msg/Twist -d 6 -o mybag3.bag
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1, y: 0.1, z: 0}, angular: {x: 0, y: 0, z: 1}}' --rate 5
ros2 bag play mybag3.bag !!!
rm -r mybag3.bag
```



# References
- ROS Paper: http://lars.mec.ua.pt/public/LAR%20Projects/Humanoid/2013_EmilioEstrelinha/Disserta%C3%A7%C3%A3o_Em%C3%ADlio_Estrelinha/Datasheets/ROS:%20an%20open-source%20Robot%20Operating%20System.pdf
- Sourceforge: http://ros.sourceforge.net
- Personal robots: http://personalrobots.sourceforge.net (Personalrobots is now hosted at code.ros.org)

