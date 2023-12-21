# costmap_depth_camera

Humble version of the plugin.
Theorems are the same from ROS1 version

## Demo: Use realsense with the point cloud published
<p float="left">
<img src="https://github.com/tsengapola/my_image_repo/blob/main/depth_camera_plugin/humble_example.gif" width="400" height="265"/>
</p>

## Required package
[navigation2_humble](https://github.com/ros-planning/navigation2/tree/humble)

## Run example
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone -b humble git@github.com:tsengapola/costmap_depth_camera.git
cd ~/ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-arg -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch costmap_depth_camera standalone_test_launch.py
```
Enjoy!
