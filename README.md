# droneAPI

How to install ardupilot_msgs:

Go into msg_ws:

rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
colcon build

sudo cp -a install/ardupilot_msgs/local/lib/python3.10/dist-packages/. /opt/ros/humble/local/lib/python3.10/dist-packages/