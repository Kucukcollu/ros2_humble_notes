# tools

## rqt

```bash 
rqt

rqt_graph
```

## rqt_console

is a GUI tool used to introspect log messages in ROS 2

```bash 
ros2 run rqt_console rqt_console
```

### Logger levels

ROS 2’s logger levels are ordered by severity:

- Fatal
- Error
- Warn
- Info*
- Debug

* : The default level is Info.

Set log level of node
```bash 
ros2 run turtlesim turtlesim_node --ros-args --log-level INFO
```