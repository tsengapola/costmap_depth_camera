/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Apola
 *********************************************************************/
#ifndef COSTMAP_DEPTH_CAMERA_OBSTACLE_LAYER_H_
#define COSTMAP_DEPTH_CAMERA_OBSTACLE_LAYER_H_

// STL
#include <time.h>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <unordered_set>

// nav2 costmap
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// observation buffer
#include <nav2_costmap_2d/observation_buffer_depth.h>
#include <nav2_costmap_2d/frustum_utils.hpp>

// messages
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "std_msgs/msg/bool.hpp"

// tf
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>

// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

/// References
/// https://navigation.ros.org/plugin_tutorials/docs/writing_new_costmap2d_plugin.html
/// https://github.com/ros-planning/navigation2_tutorials


namespace nav2_costmap_2d
{
  class DepthCameraObstacleLayer : public nav2_costmap_2d::CostmapLayer
  {
  public:
    
    DepthCameraObstacleLayer(void);
    virtual ~DepthCameraObstacleLayer(void);
    
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset(void);
    virtual bool isClearable() {return false;}

    /// @brief  A callback to handle buffering PointCloud2 messages
    /// @param message The message returned from a message notifier
    /// @param buffer A pointer to the observation buffer to update
     
    void pointCloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message,
                             const std::shared_ptr<nav2_costmap_2d::ObservationBufferDepth>& buffer);

    /// for testing purposes
    void addStaticObservation(nav2_costmap_2d::ObservationDepth& obs, bool marking, bool clearing);
    void clearStaticObservations(bool marking, bool clearing);

  protected:

  /// @brief  Get the observations used to mark space
  /// @param marking_observations A reference to a vector that will be populated with the observations
  /// @return True if all the observation buffers are current, false otherwise
  
  bool getMarkingObservations(std::vector<nav2_costmap_2d::ObservationDepth>& marking_observations) const;

  /// @brief  Get the observations used to clear space
  /// @param clearing_observations A reference to a vector that will be populated with the observations
  /// @return True if all the observation buffers are current, false otherwise
  bool getClearingObservations(std::vector<nav2_costmap_2d::ObservationDepth>& clearing_observations) const;

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;

  bool footprint_clearing_enabled_;
  
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                       double* max_x, double* max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::vector<std::shared_ptr<message_filters::SubscriberBase<rclcpp_lifecycle::LifecycleNode>>> observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<std::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBufferDepth> > observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBufferDepth> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBufferDepth> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  // Used only for testing purposes
  std::vector<nav2_costmap_2d::ObservationDepth> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_;
  //dynamic_reconfigure::Server<nav2_costmap_2d::DepthCameraPluginConfig> *dsrv_;

  int combination_method_;

private:
  /// Publishers for debugging
  /// http://wiki.ros.org/pcl/Overview [publish pcl<T> without conversion]
  /// https://answers.ros.org/question/312587/generate-and-publish-pointcloud2-in-ros2/ [pcl to sensor_msgs]

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frustum_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr marking_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_pub_;

  sensor_msgs::msg::PointCloud2::SharedPtr frustum_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr marking_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cluster_msg_;

  /// Publish frustums for visualization
  void pubFrustum(std::vector<nav2_costmap_2d::ObservationDepth>& observations);

  /// Clearing mechanism
  void ClearMarkingbyKdtree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
                            std::vector<nav2_costmap_2d::ObservationDepth>& observations,
                            double robot_x, double robot_y);

  /// Marking mechanism
  void ProcessCluster(std::vector<nav2_costmap_2d::ObservationDepth>& observations,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster_cloud, 
                      double* min_x, double* min_y, double* max_x, double* max_y);

  /// For boundary gurantee, to prevent crash/seg fault.
  bool isValid(unsigned int mx, unsigned int my);

  /// Voxel setting
  double voxel_resolution_;
  double marking_height_above_ground_;
  double marking_height_under_ground_;

  //// Voxel structure
  std::map<unsigned int, std::map<int, float> > pc_3d_map_;
  std::map<std::pair<int, int>, std::map<int, float> > pc_3d_map_global_;
  std::pair <std::map<int, float>::iterator, bool> insert_ptr_;

  /// For euclidean distance segmentation
  double ec_seg_distance_;
  int ec_cluster_min_size_;
  int size_of_cluster_rejection_;
    
  ///A point within this radius will be cleared if there is no neighbor point
  double check_radius_;
  int number_clearing_threshold_;
  
  /// Depth camera usually has blind distance, when something is too close to depth camera, no pointcloud will be detected.
  /// Without any pointcloud, markings will be falsely cleared.
  /// Enable this flag if depth camera can always sees something when working. 
  /// (ex: my case is that my depth cam will always see floor, so I set this to true.)

  bool enable_near_blocked_protection_;
  int number_points_considered_as_blocked_;

  
  /// Force clearing distance: This is usually used for tf transform is a bit slow causing marking get into robot body.
  /// Frame is based on base_link
  double forced_clearing_distance_;

  /// Relevant variable for new features --> marking/clearing using global frame
  bool use_global_frame_to_mark_;

  void worldToIntIndex(double wx, double wy, int& mx, int& my, double resolution) const;
  void intIndexToWorld(double& wx, double& wy, int mx, int my, double resolution) const;

  bool has_costmap_initial_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_obstacle_layer_sub_;
  void enableObstacleLayerCB(const std_msgs::msg::Bool::SharedPtr msg);
  bool restricted_;

};      /// class DepthCameraObstacleLayer
}       /// namespace nav2_costmap_2d 
#endif  /// COSTMAP_DEPTH_CAMERA_OBSTACLE_LAYER_H_