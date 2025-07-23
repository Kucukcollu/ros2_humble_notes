# ROS2 Basics

## Nodes

Nodes send data over topics using messages.

## Topics

### Publisher

### Subscriber

### Messages

**geometry_msgs/msg/Twist**
```msg
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z

```

## Services

Services are based on a call-and-response model versus the publisher-subscriber model of topics.

While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.

There can be many service clients using the same service. But there can only be one service server for a service.

### Service Server

### Service Client

### Services

```srv
request
---
response
```

**turtlesim/action/RotateAbsolute**

```srv
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
```

**turtlesim/srv/Spawn**
```srv
float32 x
float32 y
float32 theta
string name # Optional.  A unique name will be created and returned if this is empty
---
string name
```

## Actions

They consist of three parts: a goal, feedback, and a result.

Actions are built on topics and services. Their functionality is similar to services, except actions can be canceled.

A robot system would likely use actions for navigation. An action goal could tell a robot to travel to a position. While the robot navigates to the position, it can send updates along the way (i.e. feedback), and then a final result message once it’s reached its destination.

### Action Server

### Action Client

### Actions

```srv
goal
---
result
---
feedback
```

## Parameters

- A parameter is a configuration value of a node.
- In ROS 2, each node maintains its own parameters.
- Every node has the parameter use_sim_time; it’s not unique to turtlesim.