# ROS2 Environment Config / Settings

## Source ROS2 Humble

```bash 
# bash (~/.bashrc)
source /opt/ros/humble/setup.bash

# zsh (~/.zshrc)
source /opt/ros/humble/setup.zsh
```

## ROS_DOMAIN_ID

```bash 
# export ROS_DOMAIN_ID=<your_domain_id>

export ROS_DOMAIN_ID=9
```

## ROS_LOCALHOST_ONLY

```bash 
# topics, services, and actions will not be visible to other computers
export ROS_LOCALHOST_ONLY=1
```
