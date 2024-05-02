# droneAPI

How to install ardupilot_msgs:

Go into msg_ws:

rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
colcon build

sudo cp -a install/ardupilot_msgs/local/lib/python3.10/dist-packages/. /opt/ros/humble/local/lib/python3.10/dist-packages/

The above only helps with fixing linting issues. Before running in a new terminal you will also need to source the workspace everytime with:

source msg_ws/install/setup.bash