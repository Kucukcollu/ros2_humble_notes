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
# list of all nodes
ros2 action list
```

## ros2 bag

## ros2 component

## ros2 daemon

## ros2 doctor

## ros2 interface

```bash 
# show msg type details
ros2 interface show rcl_interfaces/msg/ParameterEvent
```

## ros2 launch

## ros2 lifecycle

## ros2 multicast

## ros2 node

```bash 
# list of all nodes
ros2 node list
```
## ros2 param

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
```
---

```bash 
# learn service type
ros2 service type /kill
```
## ros2 topic

## ros2 wtf