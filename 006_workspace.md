# workspace

## create ros2 workspace

```bash 
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## Install packages to src folder

```bash 
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble
```

## resolve dependencies

```bash 
cd ~/ros2_ws 

rosdep install -i --from-path src --rosdistro humble -y
```