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

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_depth_camera/observation_buffer.h>

#include <nav_msgs/OccupancyGrid.h>

#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_depth_camera/DepthCameraPluginConfig.h>
#include <costmap_2d/footprint.h>

/* This header is used for publish the pcl without converting to pointcloud2 msg */
#include <pcl_ros/point_cloud.h>

/* Include depth camera observation to use functions related to frustums */
#include <costmap_depth_camera/frustum_utils.hpp>

/* This is used for kd-tree search*/
#include <pcl/kdtree/kdtree_flann.h>

/*This is for euclidean distance segmentation*/
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace costmap_depth_camera
{

class DepthCameraObstacleLayer : public costmap_2d::CostmapLayer
{
public:
  DepthCameraObstacleLayer()
  {
    costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
  }

  virtual ~DepthCameraObstacleLayer();
  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                            double* max_x, double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

  virtual void activate();
  virtual void deactivate();
  virtual void reset();

  /**
   * @brief  A callback to handle buffering PointCloud2 messages
   * @param message The message returned from a message notifier
   * @param buffer A pointer to the observation buffer to update
   */
  void pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                           const boost::shared_ptr<costmap_depth_camera::ObservationBuffer>& buffer);

  // for testing purposes
  void addStaticObservation(costmap_depth_camera::Observation& obs, bool marking, bool clearing);
  void clearStaticObservations(bool marking, bool clearing);

protected:
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  Get the observations used to mark space
   * @param marking_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getMarkingObservations(std::vector<costmap_depth_camera::Observation>& marking_observations) const;

  /**
   * @brief  Get the observations used to clear space
   * @param clearing_observations A reference to a vector that will be populated with the observations
   * @return True if all the observation buffers are current, false otherwise
   */
  bool getClearingObservations(std::vector<costmap_depth_camera::Observation>& clearing_observations) const;

  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool footprint_clearing_enabled_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, 
                       double* max_x, double* max_y);

  std::string global_frame_;  ///< @brief The global frame for the costmap

  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
  std::vector<boost::shared_ptr<costmap_depth_camera::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors
  std::vector<boost::shared_ptr<costmap_depth_camera::ObservationBuffer> > marking_buffers_;  ///< @brief Used to store observation buffers used for marking obstacles
  std::vector<boost::shared_ptr<costmap_depth_camera::ObservationBuffer> > clearing_buffers_;  ///< @brief Used to store observation buffers used for clearing obstacles

  // Used only for testing purposes
  std::vector<costmap_depth_camera::Observation> static_clearing_observations_, static_marking_observations_;

  bool rolling_window_;
  dynamic_reconfigure::Server<costmap_depth_camera::DepthCameraPluginConfig> *dsrv_;

  int combination_method_;

private:

  /*Publishers for visualization*/
  ros::Publisher frustum_pub_;
  ros::Publisher voxel_filtered_pub_;
  ros::Publisher marking_pub_;
  ros::Publisher cluster_pub_;

  void reconfigureCB(costmap_depth_camera::DepthCameraPluginConfig &config, uint32_t level);

  /*publish frustums for visualization*/
  void pubFrustum(std::vector<costmap_depth_camera::Observation>& observations);

  /*Clearing mechanism*/
  void ClearMarkingbyKdtree(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
    std::vector<costmap_depth_camera::Observation>& observations,
    double robot_x, double robot_y);

  /*Marking mechanism*/
  void ProcessCluster(
    std::vector<costmap_depth_camera::Observation>& observations, 
    pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster_cloud, 
    double robot_x, double robot_y,
    double* min_x, double* min_y, double* max_x, double* max_y);

  /*For boundary gurantee, to prevent crash/seg fault.*/
  bool isValid(unsigned int mx, unsigned int my);

  /*voxel setting*/
  double voxel_resolution_;
  double marking_height_above_ground_, marking_height_under_ground_;

  /*Voxel structure*/
  std::map<unsigned int, std::map<int, float> > pc_3d_map_;
  std::pair <std::map<int, float>::iterator, bool> insert_ptr_;

  /*For euclidean distance segmentation*/
  double ec_seg_distance_;
  int ec_cluster_min_size_;
  int size_of_cluster_rejection_;
    
  /*A point within this radius will be cleared if there is no neighbor point*/
  double check_radius_;
  int number_clearing_threshold_;
  
  /*
  Depth camera usually has blind distance, when something is too close to depth camera, no pointcloud will be detected.
  Without any pointcloud, markings will be falsely cleared.
  Enable this flag if depth camera can always sees something when working. (ex: my case is that my depth cam will always see floor, so I set this to true.)
  */
  bool enable_near_blocked_protection_;
  int number_points_considered_as_blocked_;

  /*
  Force clearing distance: This is usually used for tf transform is a bit slow causing marking get into robot body.
  Frame is based on base_link
  */
  double forced_clearing_distance_;

};

}  // namespace costmap_depth_camera

#endif  // COSTMAP_DEPTH_CAMERA_OBSTACLE_LAYER_H_
