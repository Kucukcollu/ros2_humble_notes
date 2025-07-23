# ROS2 Command-line Tool

## ros2 manual

```bash 
$ ros2 --help

# usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...

# ros2 is an extensible command-line tool for ROS 2.

# options:
#   -h, --help            show this help message and exit
#   --use-python-default-buffering
#                         Do not force line buffering in stdout and instead use the python default buffering, which might be
#                         affected by PYTHONUNBUFFERED/-u and depends on whatever stdout is interactive or not

# Commands:
#   action     Various action related sub-commands
#   bag        Various rosbag related sub-commands
#   component  Various component related sub-commands
#   daemon     Various daemon related sub-commands
#   doctor     Check ROS setup and other potential issues
#   interface  Show information about ROS interfaces
#   launch     Run a launch file
#   lifecycle  Various lifecycle related sub-commands
#   multicast  Various multicast related sub-commands
#   node       Various node related sub-commands
#   param      Various param related sub-commands
#   pkg        Various package related sub-commands
#   run        Run a package specific executable
#   security   Various security related sub-commands
#   service    Various service related sub-commands
#   topic      Various topic related sub-commands
#   wtf        Use `wtf` as alias to `doctor`

#   Call `ros2 <command> -h` for more detailed usage.

```

## ros2 action

```bash 
# list of all actions
ros2 action list

# list of all actions with type
ros2 action list -t

# see details of action
ros2 action info /turtle1/rotate_absolute

# sent action goal
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/ RotateAbsolute '{theta: 1.57}'
```

## ros2 bag

```bash 
# record a single topic
ros2 bag record /turtlesim1/turtle1/cmd_vel

# record a ros2 bag with specific name
ros2 bag record -o turtlesim_bag /turtlesim/turtle1/cmd_vel /turtlesim1/turtle1/pose

# record all topics
ros2 bag record -a

# details of bag
ros2 bag info turtlesim_bag

# play ros2 bag
ros2 bag play turtlesim_bag
```

## ros2 component

## ros2 daemon

## ros2 doctor

## ros2 interface

```bash 
# show msg type details
ros2 interface show geometry_msgs/msg/Twist

# show srv type details
ros2 interface show turtlesim/srv/Kill

# show action type details
ros2 interface show turtlesim/action/RotateAbsolute

# interface prototype ---> see dummy interface
ros2 interface proto sensor_msgs/msg/Imu
```

## ros2 launch

```bash 
# basic usage
ros2 launch turtlesim multisim.launch.py
```

## ros2 lifecycle

## ros2 multicast

## ros2 node

```bash 
# list of all nodes
ros2 node list

# see node info
ros2 node info /turtlesim
```

## ros2 param

```bash 
# list of all params
ros2 param list

# display current value
ros2 param get /turtlesim background_b

# change current value
ros2 param set /turtlesim background_b 50

# all of a node's current paramater values
ros2 param dump /turtlesim

# save all of a node's current parameter values
ros2 param dump /turtlesim > parameters.yaml

# load params to currently running node
ros2 param load /turtlesim turtlesim.yaml
```

## ros2 pkg

```bash 
# a list of turtlesim’s executables
ros2 pkg executables turtlesim
```

## ros2 run

```bash 
# run a ros2 node
# ros2 run <pakcage_name> <node_name>
ros2 run turtlesim turtlesim_node

# remap node name
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=/my_cute_turtle

# load param @ startup
ros2 run turtlesim turtlesim_node --ros-args --params-file params.yaml

#Set log level of node
ros2 run turtlesim turtlesim_node --ros-args --log-level INFO
```

---

```bash 
# remap sub pub topic while running ros2 node
# use turtle2/cmd_vel instead of turtle1/cmd_vel
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

## ros2 security

## ros2 service

```bash 
# list of all nodes
ros2 service list

# learn service type
ros2 service type /kill

ros2 service type /clear
# std_srvs/srv/Empty

# The Empty type means the service call sends no data when making a request and receives no data when receiving a response.

# list all services with types
ros2 service list -t

# find specific type of services
ros2 service find std_srvs/srv/Empty

# call service
ros2 service call /clear std_srvs/srv/Empty

ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```
## ros2 topic

```bash 
# list all topics
ros2 topic list

# list all topics with their types
ros2 topic list -t

# see topic data
ros2 topic echo /turtle1/cmd_vel

# see topic pubs and subs
ros2 topic info /turtle1/cmd_vel

# publish messages (require YAML format) by default publishes @1 Hz
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# publish msg at different frequencies
ros2 topic pub --rate 5 /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# publish only once and wait 2 subscriber
ros2 topic pub --once -w 2 /turtle1/cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"

# fill header time auto
ros2 topic pub /pose geometry_msgs/msg/PoseStamped \
'{header: "auto", pose: {position: {x: 1.0, y: 2.0, z: 3.0}}}'

# if msg does not include header time (builtin_interfaces/msg/Time)
ros2 topic pub /reference sensor_msgs/msg/TimeReference \
'{header: "auto", time_ref: "now", source: "dumy"}'

# see frequency of a topic
ros2 topic hz /turtle1/pose

# see bandwidth
ros2 topic bw /turtle1/pose

# see specific type of topics
ros2 topic find geometry_msgs/msg/Twist

```

## ros2 wtf