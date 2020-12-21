# costmap_depth_camera
This is a costmap plugin for costmap_2d pkg

Considering ray casting method can not satisfy sparse 3D space problem of clearing. This plugin is based on kd-tree search to clear the markings.

This plugin comprises two parts:
1. Marking pointcloud and store it using std::map.
2. Clearing marked pointcloud using kd-tree search which is from Point Cloud Library (PCL).

## Required package
[costmap_2d](http://wiki.ros.org/costmap_2d)
  
## Demo: Use realsense with the point cloud published
<p float="left">
<img src="https://github.com/tsengapola/my_image_repo/blob/main/depth_camera_plugin/Use%20depth%20camera%20(realsense)%20to%20detect%20the%20bar%20across%20the%20cones.gif" width="400" height="265"/>
<img src="https://github.com/tsengapola/my_image_repo/blob/main/depth_camera_plugin/op-costmap-depth-cam.gif" width="400" height="265"/>
</p>

## Launch Example

* Note that costmap_depth_camera.launch subscribe imu data from realsense D435i and create TF from map-->base_link. Change anything to meet your system requirements. If you have point cloud published, realsense is not needed.
```
roslaunch realsense2_camera rs_camera.launch
roslaunch costmap_depth_camera costmap_depth_camera.launch 
```

## .yaml example of a workable setting:

```
costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 5
  static_map: false
  rolling_window: true
  width: 20
  height: 20
  resolution: 0.05  
  transform_tolerance: 1.0
  footprint: [[0.46,0.25], [0.36,0.37], [0.33,0.39], [-0.39,0.4], [-0.48,0], [-0.39,-0.4], [0.33,-0.39], [0.36,-0.37], [0.46,-0.25], [0.49,0]]

  plugins:
    - {name: static, type: 'costmap_2d::StaticLayer'}
    - {name: 3DPerception, type: 'costmap_depth_camera::DepthCameraObstacleLayer'}
    - {name: inflation, type: 'costmap_2d::InflationLayer'}

  static:
    map_topic: /map

  inflation:
    inflation_radius: 1.0
    cost_scaling_factor: 0.5

  3DPerception:
    ec_seg_distance: 0.2
    ec_cluster_min_size: 5
    size_of_cluster_rejection: 5
    voxel_resolution: 0.01
    check_radius: 0.1
    enable_near_blocked_protection: false
    number_points_considered_as_blocked: 5
    forced_clearing_distance: 0.1
    observation_sources: depth_cam #depth_cam_left
    depth_cam: {sensor_frame: camera_link, topic: /camera/depth/color/points, expected_update_rate: 0.3, FOV_W: 1.0, FOV_V: 0.9, min_detect_distance: 0.15, max_detect_distance: 2.0, min_obstacle_height: 0.08}
    #depth_cam_left: {sensor_frame: camera_link_left, topic: /camera_left/depth/color/points, expected_update_rate: 0.3, FOV_W: 1.0, FOV_V: 0.9, min_detect_distance: 0.15, max_detect_distance: 2.0, min_obstacle_height: 0.08}
```

## Parameters Description (many of them are supported in dynamic reconfigure)

* ec_seg_distance: 0.2
* ec_cluster_min_size: 5
* size_of_cluster_rejection: 5

> Implement [Euclidean Cluster Extraction](https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html#cluster-extraction).
For ec_seg_distance, it is useful when some object is black and small, if it is connect to a bigger object, then size_of_cluster_rejection will not reject that small piece of black object.

> For size_of_cluster_rejection, this is used to ignore small object in the space. It seems even though ec_cluster_min_size is set to 5, the Euclidean Cluster Extraction will still output the cluster small than 5 points. Therefore, we can use this function to ignore them (will not add in markings).

* voxel_resolution: 0.01

> Usually, 1 cm voxel height is enough.

* number_clearing_threshold: 2
* check_radius: 0.1

> A kd-tree radius search of a marked point will be used to find the neighbors, if number of neighbors is higher than number_clearing_threshold. The marked point will be cleared.
For example, a marked point  only has one neighbor within 0.1 meter. So the marked point will be cleared.

* forced_clearing_distance: 0.1

> If a point appears within this distance, it will be cleared. The distance is calculated from the point to robot_base_frame. 
> For example, a noise point appear in the robot footprint will be cleared.

* number_points_considered_as_blocked: 5
* enable_near_blocked_protection: true

> If number of points in point cloud is less than 'number_points_considered_as_blocked', clearing mechanism is skip.
For example, my robot can always see floor and thus the point cloud will always has points. If a new message arrive with few points, we simply assume the depth camera is blocked by something (mostly people's hand, they like to wave their hands in front of robot).
> When depth camera detect nothing, it will clear all markings in the frustum, by enabling this protect function, the false clearing can be prevent.

## TF requirement

Make sure the sensor_frame in yaml file follows the setting (in general case of realsense is camera_link), i.e.: z axes points to sky, x axes points to moving direction.
The orange points show the frustum of my camera.

<img src="https://github.com/tsengapola/my_image_repo/blob/main/depth_camera_plugin/frustum_axis.png" width="450" height="270"/>

## Acknowledge
Many thanks to [Uniring MAX - The leader of robotic floor scrubber](https://portalimages.blob.core.windows.net/products/pdfs/fsngkduv_UNIRING-MAX_AE_FINAL.pdf)
