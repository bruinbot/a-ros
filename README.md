# a-ros

## BruinBot ROS Architecture
We use ROS2 Foxy on Ubuntu 20.04 LTS.

Dynamic Transforms:
```
sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui
```

### Implement package to your robot hardware or developer PC

In your home directory (~), clone the repo:
```
mkdir robot_ws/src
cd src
git clone git@github.com:bruinbot/a-ros.git
```

Source ROS2 Foxy:
```
source /opt/ros/foxy/setup.bash
```

Build our files:
```
cd ../.. && colcon build --symlink-install
```

Source BruinBot package and launch:
```
source install/setup.bash
ros2 launch bruinbot_pkg talker.launch.py
```
