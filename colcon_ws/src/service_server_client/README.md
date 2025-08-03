# service_server_client

## build
```bash
# install repo
git clone https://github.com/Kucukcollu/ros2_humble_notes.git

# move through ros2 workspace
cd ros2_humble_notes/colcon_ws

colcon build
```

## run
```bash
# source workspace
source install/setup.bash

# run service server
ros2 run service_cerver_client service_server

## run service client from terminal
ros2 service call /add_two_numbers example_interfaces/srv/AddTwoInts "{a: 4, b: 8}"

# OR run service client
ros2 run service_cerver_client service_client 4 8
```

