# droneAPI
#### A generic integration library for ROS2 with Ardupilot

# How to install ardupilot_msgs:

Go into msg_ws and build the library:

```
cd msg_ws
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src
colcon build
```

Then we are copying the built files over into our main install for ROS so our python linter can find them:  
```
sudo cp -a install/ardupilot_msgs/local/lib/python3.10/dist-packages/. /opt/ros/humble/local/lib/python3.10/dist-packages/
```

# NOTE:  
The above only helps with fixing linting issues.  
Before running in a new terminal you will also need to source the workspace everytime with:
```
source msg_ws/install/setup.bash
```

### Copter Mode Switch
https://mavlink.io/en/messages/ardupilotmega.html#COPTER_MODE