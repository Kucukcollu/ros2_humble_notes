# ROS2 Basics

## Nodes

Nodes send data over topics using messages.

## Messages

```bash 
$ ros2 topic list
/turtle1/cmd_vel [geometry_msgs/msg/Twist]
# topic name [message type]
```
This means that in the package geometry_msgs there is a msg called Twist.

## Topics

### Publisher

### Subscriber

## Services

Services are based on a call-and-response model versus the publisher-subscriber model of topics.

While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

There can be many service clients using the same service. But there can only be one service server for a service.

### Example service
```bash
ros2 interface show turtlesim/srv/Spawn

float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

### Service Server

### Service Client

## Parameters

- A parameter is a configuration value of a node.
- In ROS 2, each node maintains its own parameters.
- Every node has the parameter use_sim_time; it’s not unique to turtlesim.

## Actions

### Action Server

### Action Client