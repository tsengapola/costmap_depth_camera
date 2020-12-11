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
#include <costmap_depth_camera/depth_camera_obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <tf2_ros/message_filter.h>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

PLUGINLIB_EXPORT_CLASS(costmap_depth_camera::DepthCameraObstacleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_depth_camera::ObservationBuffer;
using costmap_depth_camera::Observation;

namespace costmap_depth_camera
{

void DepthCameraObstacleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_), g_nh;
  rolling_window_ = layered_costmap_->isRolling();

  bool track_unknown_space;
  nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
  if (track_unknown_space)
    default_value_ = NO_INFORMATION;
  else
    default_value_ = FREE_SPACE;

  DepthCameraObstacleLayer::matchSize();
  current_ = true;

  /*
  Publishers for visualization
  */
  frustum_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("frustum", 2);
  marking_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("marking_pc", 2);
  cluster_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("clustered_pc", 2);

  /*
  To be refined
  */
  nh.param("enable_near_blocked_protection", enable_near_blocked_protection_, true);
  nh.param("number_points_considered_as_blocked", number_points_considered_as_blocked_, 5);
  nh.param("forced_clearing_distance", forced_clearing_distance_, 0.1);
  nh.param("ec_seg_distance", ec_seg_distance_, 0.2);
  nh.param("ec_cluster_min_size", ec_cluster_min_size_, 5);
  nh.param("size_of_cluster_rejection", size_of_cluster_rejection_, 5);
  nh.param("voxel_resolution", voxel_resolution_, 0.01);
  nh.param("check_radius", check_radius_, 0.1);
  nh.param("number_clearing_threshold", number_clearing_threshold_, 2);
  

  marking_height_under_ground_ = 100.0;
  marking_height_above_ground_ = -100.0;

  global_frame_ = layered_costmap_->getGlobalFrameID();
  double transform_tolerance;
  nh.param("transform_tolerance", transform_tolerance, 0.2);

  std::string topics_string;
  // get the topics that we'll subscribe to from the parameter server
  nh.param("observation_sources", topics_string, std::string(""));
  ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

  // now we need to split the topics based on whitespace which we can use a stringstream for
  std::stringstream ss(topics_string);

  std::string source;
  while (ss >> source)
  {
    ros::NodeHandle source_node(nh, source);

    // get the parameters for the specific topic
    double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
    std::string topic, sensor_frame, data_type;
    bool inf_is_valid, clearing, marking;
    double FOV_V, FOV_W, min_detect_distance, max_detect_distance;

    source_node.param("topic", topic, source);
    source_node.param("sensor_frame", sensor_frame, std::string(""));
    source_node.param("observation_persistence", observation_keep_time, 0.0);
    source_node.param("expected_update_rate", expected_update_rate, 0.0);
    source_node.param("min_obstacle_height", min_obstacle_height, 0.0);
    source_node.param("max_obstacle_height", max_obstacle_height, 2.0);
    source_node.param("inf_is_valid", inf_is_valid, false);
    source_node.param("clearing", clearing, false);
    source_node.param("marking", marking, true);

    source_node.param("FOV_V", FOV_V, 1.0);
    source_node.param("FOV_W", FOV_W, 1.5);
    source_node.param("min_detect_distance", min_detect_distance, 0.15);
    source_node.param("max_detect_distanxe", max_detect_distance, 2.5);

    /*Update minimum height and maximum height for overall marking*/
    marking_height_above_ground_ = std::max(marking_height_above_ground_,max_obstacle_height);
    marking_height_under_ground_ = std::min(marking_height_under_ground_,min_obstacle_height);
    ROS_WARN("Update minimum height of markings to: %.2f, maximum height of markings to %.2f", marking_height_under_ground_, marking_height_above_ground_);

    std::string raytrace_range_param_name, obstacle_range_param_name;

    // get the obstacle range for the sensor
    double obstacle_range = 2.5;
    if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
    {
      source_node.getParam(obstacle_range_param_name, obstacle_range);
    }

    // get the raytrace range for the sensor
    double raytrace_range = 3.0;
    if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
    {
      source_node.getParam(raytrace_range_param_name, raytrace_range);
    }

    ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
              sensor_frame.c_str());

    // create an observation buffer
    observation_buffers_.push_back(
        boost::shared_ptr < ObservationBuffer
            > (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
                                     max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
                                     sensor_frame, transform_tolerance,
                                     FOV_V, FOV_W, min_detect_distance, max_detect_distance)));

    // check if we'll add this buffer to our marking observation buffers
    if (marking)
      marking_buffers_.push_back(observation_buffers_.back());

    // check if we'll also add this buffer to our clearing observation buffers
    if (clearing)
      clearing_buffers_.push_back(observation_buffers_.back());

    ROS_DEBUG(
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

    // create a callback for the topic

    boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
        > sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 5));

    if (inf_is_valid)
    {
      ROS_WARN("depth_camera_obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
    }

    boost::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::PointCloud2>
    > filter(new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 5, g_nh));
    filter->registerCallback(
        boost::bind(&DepthCameraObstacleLayer::pointCloud2Callback, this, _1, observation_buffers_.back()));

    observation_subscribers_.push_back(sub);
    observation_notifiers_.push_back(filter);
    

    if (sensor_frame != "")
    {
      std::vector < std::string > target_frames;
      target_frames.push_back(global_frame_);
      target_frames.push_back(sensor_frame);
      observation_notifiers_.back()->setTargetFrames(target_frames);
    }
  }

  dsrv_ = NULL;
  setupDynamicReconfigure(nh);
}

void DepthCameraObstacleLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
{
  dsrv_ = new dynamic_reconfigure::Server<costmap_depth_camera::DepthCameraPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_depth_camera::DepthCameraPluginConfig>::CallbackType cb = boost::bind(
      &DepthCameraObstacleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

DepthCameraObstacleLayer::~DepthCameraObstacleLayer()
{
    if (dsrv_)
        delete dsrv_;
}
void DepthCameraObstacleLayer::reconfigureCB(costmap_depth_camera::DepthCameraPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
  footprint_clearing_enabled_ = config.footprint_clearing_enabled;
  ec_seg_distance_ = config.ec_seg_distance;
  ec_cluster_min_size_ = config.ec_cluster_min_size;
  voxel_resolution_ = config.voxel_resolution;
  check_radius_ = config.check_radius;
  size_of_cluster_rejection_ = config.size_of_cluster_rejection;
  combination_method_ = config.combination_method;
}

void DepthCameraObstacleLayer::pointCloud2Callback(const sensor_msgs::PointCloud2ConstPtr& message,
                                                const boost::shared_ptr<ObservationBuffer>& buffer)
{
  // buffer the point cloud
  buffer->lock();
  buffer->bufferCloud(*message);
  buffer->unlock();
}

void DepthCameraObstacleLayer::pubFrustum(std::vector<costmap_depth_camera::Observation>& observations){
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr pub_frustum(new pcl::PointCloud<pcl::PointXYZI>);
  int cnt = 1;
  for (std::vector<costmap_depth_camera::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const costmap_depth_camera::Observation& obs = *it;
    for (unsigned int i = 0; i < obs.frustum_->size(); ++i)
    {
      /* 
      Transform from pcl::XYZ to pcl::XYZI,
      so we can use intensity to distinguish different frustum (camera).
      */
      pcl::PointXYZI rgb_pt;
      rgb_pt.x = obs.frustum_->points[i].x;
      rgb_pt.y = obs.frustum_->points[i].y;
      rgb_pt.z = obs.frustum_->points[i].z;
      rgb_pt.intensity = cnt*150;
      //ROS_DEBUG("%.2f,%.2f,%.2f",rgb_pt.x,rgb_pt.y,rgb_pt.z);
      pub_frustum->push_back(rgb_pt);
    }
    cnt++;
    
  }

  pcl_conversions::toPCL(ros::Time::now(), pub_frustum->header.stamp);
  pub_frustum->header.frame_id = "map";
  frustum_pub_.publish(pub_frustum);
  
}

void DepthCameraObstacleLayer::ClearMarkingbyKdtree(
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
  std::vector<costmap_depth_camera::Observation>& observations,
  double robot_x, double robot_y){

  unsigned int mx, my;
  double wx,wy;
  unsigned int index;
  int mmx, mmy;
  pcl::PointXYZI searchPoint;

  pcl::PointCloud<pcl::PointXYZI>::Ptr marking(new pcl::PointCloud<pcl::PointXYZI>);

  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_gbl;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  bool bypass_clearing = false;
  if(enable_near_blocked_protection_){
    if(cloud_in->points.size()<number_points_considered_as_blocked_){
      ROS_WARN_THROTTLE(30.0, "Blocked by something, clearing mechanism is skipped.");
      bypass_clearing = true;
    }
  }

  bool clear_all_marking_in_this_frame = false;
  if(cloud_in->points.size()>5){
    kdtree_gbl.setInputCloud (cloud_in);    
  }
  else{
    if(!enable_near_blocked_protection_){
      clear_all_marking_in_this_frame = true; //We really see nothing in the space
    }
    else{
      bypass_clearing = true;
    }
  }


  bool is_marking_sub = marking_pub_.getNumSubscribers()>0;

  /*
  Prepare for frustum_utils
  */
  FrustumUtils frustum_utils(&observations);
  
  for(auto it_3d_map=pc_3d_map_.begin();it_3d_map!=pc_3d_map_.end();it_3d_map++){
    
    indexToCells((*it_3d_map).first, mx, my);
    mapToWorld(mx, my, wx, wy);

    bool is_in_FRUSTUM = false;
    bool is_attach_FRUSTUM = false;
    bool is_check_clear = false;
    //if marked point cloud is n meters from robot, we just skip, because it is out of our frustum 
    if(hypot(wx-robot_x,wy-robot_y)>10.0 && !is_marking_sub){
      continue;
    }

    for(auto it = (*it_3d_map).second.begin(), next_it = it; it != (*it_3d_map).second.end(); it = next_it) {
      ++next_it;

      searchPoint.x = wx;
      searchPoint.y = wy;
      searchPoint.z = (*it).first*voxel_resolution_;
      searchPoint.intensity = (*it).second;

      if(is_marking_sub){
        marking->push_back(searchPoint);
      }

      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      double pc_dis = hypot(wx-robot_x,wy-robot_y);
      is_in_FRUSTUM = frustum_utils.isInsideFRUSTUMs(searchPoint);

      if(is_in_FRUSTUM && clear_all_marking_in_this_frame){
        /*Nothing is detected, clear all markings.*/
        //ROS_WARN("Clear all markinging in this frame.");
        (*it_3d_map).second.erase(it);
      }
      else if(is_in_FRUSTUM && !bypass_clearing && pc_dis<=forced_clearing_distance_){
        //ROS_WARN("CFOV Erase : %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
        //if(pc_dis<=forced_clearing_distance_)
        //  ROS_WARN("Clear by Footprint: %.2f",pc_dis);
        (*it_3d_map).second.erase(it);
      }
      else if (is_in_FRUSTUM && !bypass_clearing) {
        if(kdtree_gbl.radiusSearch (searchPoint, check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance)<number_clearing_threshold_){
          //ROS_WARN("Erase by kdtree: %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
          (*it_3d_map).second.erase(it);
        }
      }
      else {
        //ROS_WARN("Pass: %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
      }
    }
  }
  
  if(is_marking_sub){
    pcl_conversions::toPCL(ros::Time::now(), marking->header.stamp);
    marking->header.frame_id = "map";
    marking_pub_.publish(marking);
  }

}

void DepthCameraObstacleLayer::ProcessCluster(std::vector<costmap_depth_camera::Observation>& observations,
                pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster_cloud, double robot_x, double robot_y, 
                double* min_x, double* min_y, double* max_x, double* max_y){


  geometry_msgs::PointStamped pt;

  unsigned int mx, my;
  double wx,wy;
  unsigned int index;
  int mmx, mmy;

  /*
  Prepare for frustum_utils
  */
  FrustumUtils frustum_utils(&observations);

  for(int i=0;i<cluster_cloud->points.size();i++){

    pt.point.x = cluster_cloud->points[i].x;
    pt.point.y = cluster_cloud->points[i].y;
    pt.point.z = cluster_cloud->points[i].z;
    //tf_->transformPoint (global_frame_, pt, tf_pt);

    wx = pt.point.x;
    wy = pt.point.y;

    pcl::PointXYZI searchPoint;
    searchPoint.x = wx;
    searchPoint.y = wy;
    searchPoint.z = cluster_cloud->points[i].z;
    
    bool is_in_FRUSTUM = frustum_utils.isInsideFRUSTUMs(searchPoint);
    bool is_attach_FRUSTUM = frustum_utils.isAttachFRUSTUMs(searchPoint);

    /*These are robus marking conditions, attachment testing usually causing boundary condition.*/
    if(!is_in_FRUSTUM || is_attach_FRUSTUM){
      continue;
    }  
    

    if (!worldToMap(wx, wy, mx, my))
    {
      ROS_DEBUG("Computing map coords failed");
      continue;
    }    
    
    unsigned int index = getIndex(mx,my);

    int h_ind = (int)round(cluster_cloud->points[i].z*(1/voxel_resolution_));
    if(mx<0 || my<0 || h_ind>(int)marking_height_above_ground_/voxel_resolution_ || h_ind<(int)marking_height_under_ground_/voxel_resolution_)
      continue;

    insert_ptr_ = pc_3d_map_[index].insert(std::pair<int, float>(h_ind, cluster_cloud->points[i].intensity));
    if(!insert_ptr_.second)//the key is already in map, put max label in it!
      pc_3d_map_[index][h_ind] = std::max(pc_3d_map_[index][h_ind],cluster_cloud->points[i].intensity);

    touch(wx, wy, min_x, min_y, max_x, max_y);

  }

}

void DepthCameraObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
                                          double* min_y, double* max_x, double* max_y)
{
  if (rolling_window_)
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!enabled_)
    return;
  useExtraBounds(min_x, min_y, max_x, max_y);
  
  /*
  ROS_INFO("ec distance: %.2f",ec_seg_distance_);
  ROS_INFO("ec seg min size: %.d",ec_cluster_min_size_);
  ROS_INFO("voxel resolution: %.2f",voxel_resolution_);
  ROS_INFO("check radius: %.2f",check_radius_);
  */

  bool current = true;
  std::vector<Observation> observations, clearing_observations;

  // get the marking observations
  current = current && getMarkingObservations(observations);

  // get the clearing observations
  //current = current && getClearingObservations(clearing_observations);

  // update the global current status
  current_ = current;

  //publish frustum for visualization
  pubFrustum(observations);

  //combine all pointcloud from all observations
  pcl::PointCloud<pcl::PointXYZI>::Ptr combined_observations(new pcl::PointCloud<pcl::PointXYZI>);
  for (std::vector<costmap_depth_camera::Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
  {
    const costmap_depth_camera::Observation& obs = *it;
    *combined_observations += *(obs.cloud_);
  }

  //Given combined pointcloud to clear the markings by kd-tree method
  ClearMarkingbyKdtree(combined_observations, observations, robot_x, robot_y);

  // For cluster pub
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered2pub(new pcl::PointCloud<pcl::PointXYZI>);
  int intensity_cnt = 100;

  if(combined_observations->points.size()>5){
    pcl::search::KdTree<pcl::PointXYZI>::Ptr pc_kdtree (new pcl::search::KdTree<pcl::PointXYZI>);
    pc_kdtree->setInputCloud (combined_observations);

    std::vector<pcl::PointIndices> cluster_indices_segmentation;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_segmentation;
    ec_segmentation.setClusterTolerance (ec_seg_distance_);
    ec_segmentation.setMinClusterSize (ec_cluster_min_size_);
    ec_segmentation.setMaxClusterSize (combined_observations->points.size());
    ec_segmentation.setSearchMethod (pc_kdtree);
    ec_segmentation.setInputCloud (combined_observations);
    ec_segmentation.extract (cluster_indices_segmentation);
    //int cls_num = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        cloud_cluster->points.push_back (combined_observations->points[*pit]); 
        /*For visualization purpose*/  
        pcl::PointXYZI i_pt;
        i_pt.x = combined_observations->points[*pit].x;
        i_pt.y = combined_observations->points[*pit].y;
        i_pt.z = combined_observations->points[*pit].z;
        i_pt.intensity = intensity_cnt;
        cloud_clustered2pub->push_back(i_pt);
      } 
      intensity_cnt += 100;
      if(cloud_cluster->points.size()<=size_of_cluster_rejection_){
        /*Do not add into markings*/
        continue;
      }
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      ProcessCluster(observations, cloud_cluster, robot_x, robot_y, min_x, min_y, max_x, max_y);
    }  
    if(cluster_pub_.getNumSubscribers()>0){
      pcl_conversions::toPCL(ros::Time::now(), cloud_clustered2pub->header.stamp);
      cloud_clustered2pub->header.frame_id = "map";
      cluster_pub_.publish(cloud_clustered2pub);
    }
  }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void DepthCameraObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                    double* max_x, double* max_y)
{
    if (!footprint_clearing_enabled_) return;
    costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
}

bool DepthCameraObstacleLayer::isValid(unsigned int mx, unsigned int my)
{
  if(mx>getSizeInCellsX() || my>getSizeInCellsY())
    return false;
  return true;
}

void DepthCameraObstacleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
    return;
  
  unsigned char* master_array = master_grid.getCharMap();
  unsigned int mx, my; 

  for(auto it_3d_map=pc_3d_map_.begin();it_3d_map!=pc_3d_map_.end();){
    
    indexToCells((*it_3d_map).first, mx, my);

    /*
    * mx+1/mx-1 check is used to solve an issue of rounding problem, due to rounding may shift the index 1 step ahead
    */
    if(isValid(mx-1,my-1) && isValid(mx-1,my+1) && isValid(mx+1,my-1) && isValid(mx+1,my+1) && !(*it_3d_map).second.empty()){
      unsigned int index = getIndex(mx,my);
      master_array[index] = std::max(costmap_2d::LETHAL_OBSTACLE, master_array[index]); //change index to global
      ++it_3d_map;
    }
    else{
      pc_3d_map_.erase(it_3d_map++);
    }
  }
  
  if (footprint_clearing_enabled_)
  {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_)
  {
    case 0:  // Overwrite
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case 1:  // Maximum
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
}

void DepthCameraObstacleLayer::addStaticObservation(costmap_depth_camera::Observation& obs, bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.push_back(obs);
  if (clearing)
    static_clearing_observations_.push_back(obs);
}

void DepthCameraObstacleLayer::clearStaticObservations(bool marking, bool clearing)
{
  if (marking)
    static_marking_observations_.clear();
  if (clearing)
    static_clearing_observations_.clear();
}

bool DepthCameraObstacleLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
{
  bool current = true;
  // get the marking observations
  for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
  {
    marking_buffers_[i]->lock();
    marking_buffers_[i]->getObservations(marking_observations);
    current = marking_buffers_[i]->isCurrent() && current;
    marking_buffers_[i]->unlock();
  }
  marking_observations.insert(marking_observations.end(),
                              static_marking_observations_.begin(), static_marking_observations_.end());
  return current;
}

bool DepthCameraObstacleLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
{
  bool current = true;
  // get the clearing observations
  for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
  {
    clearing_buffers_[i]->lock();
    clearing_buffers_[i]->getObservations(clearing_observations);
    current = clearing_buffers_[i]->isCurrent() && current;
    clearing_buffers_[i]->unlock();
  }
  clearing_observations.insert(clearing_observations.end(),
                              static_clearing_observations_.begin(), static_clearing_observations_.end());
  return current;
}

void DepthCameraObstacleLayer::activate()
{
  // if we're stopped we need to re-subscribe to topics
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->subscribe();
  }

  for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
  {
    if (observation_buffers_[i])
      observation_buffers_[i]->resetLastUpdated();
  }
}
void DepthCameraObstacleLayer::deactivate()
{
  for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
  {
    if (observation_subscribers_[i] != NULL)
      observation_subscribers_[i]->unsubscribe();
  }
  pc_3d_map_.clear();
}

void DepthCameraObstacleLayer::reset()
{
    deactivate();
    resetMaps();
    current_ = true;
    activate();
}

}  // namespace costmap_depth_camera
