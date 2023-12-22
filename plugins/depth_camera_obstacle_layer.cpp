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
#include <string>
#include <unordered_map>
#include <memory>
#include <vector>
#include <costmap_depth_camera/depth_camera_obstacle_layer.h>
#include <costmap_depth_camera/frustum_utils.hpp>
#include <tf2_ros/message_filter.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::ObservationBufferDepth;
using nav2_costmap_2d::ObservationDepth;

#define DEBUG 0

namespace nav2_costmap_2d
{
  /////////////////////////////////////////////////////////////////
  DepthCameraObstacleLayer::DepthCameraObstacleLayer(void)
  ////////////////////////////////////////////////////////////////
  {
    //supress the no intensity found log
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    //costmap_ = NULL;
  }
  
  ////////////////////////////////////////////////////////////////
  DepthCameraObstacleLayer::~DepthCameraObstacleLayer(void)
  ////////////////////////////////////////////////////////////////
  {
  }

  ///////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::onInitialize(void)
  {
    restricted_ = false;
    RCLCPP_INFO(logger_, "%s being initialized as DepthCameraObstacleLayer!", getName().c_str());
    
    auto node = node_.lock();
    rolling_window_ = layered_costmap_->isRolling();
    has_costmap_initial_ = false;

    bool track_unknown_space;
    declareParameter("track_unknown_space", rclcpp::ParameterValue(layered_costmap_->isTrackingUnknown()));
    node->get_parameter(name_ + ".track_unknown_space", track_unknown_space);
    
    if (track_unknown_space)
      default_value_ = NO_INFORMATION;
    else
      default_value_ = FREE_SPACE;

    DepthCameraObstacleLayer::matchSize();
    current_ = true;

    ///Publishers for visualization
    auto pub_opt = rclcpp::PublisherOptions();
    auto sub_opt = rclcpp::SubscriptionOptions();
    
    frustum_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("frustum", rclcpp::QoS(1), pub_opt);
    marking_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("marking_pc", rclcpp::QoS(1), pub_opt);
    cluster_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_pc", rclcpp::QoS(1), pub_opt);

    declareParameter("enable_near_blocked_protection", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".enable_near_blocked_protection", enable_near_blocked_protection_);

    declareParameter("number_points_considered_as_blocked", rclcpp::ParameterValue(5));
    node->get_parameter(name_ + ".number_points_considered_as_blocked", number_points_considered_as_blocked_);

    declareParameter("forced_clearing_distance", rclcpp::ParameterValue(0.1));
    node->get_parameter(name_ + ".forced_clearing_distance", forced_clearing_distance_);

    declareParameter("ec_seg_distance", rclcpp::ParameterValue(0.2));
    node->get_parameter(name_ + ".ec_seg_distance", ec_seg_distance_);

    declareParameter("ec_cluster_min_size", rclcpp::ParameterValue(5));
    node->get_parameter(name_ + ".ec_cluster_min_size", ec_cluster_min_size_);

    declareParameter("size_of_cluster_rejection", rclcpp::ParameterValue(5));
    node->get_parameter(name_ + ".size_of_cluster_rejection", size_of_cluster_rejection_);
    
    declareParameter("voxel_resolution", rclcpp::ParameterValue(0.15));
    node->get_parameter(name_ + ".voxel_resolution", voxel_resolution_);

    declareParameter("check_radius", rclcpp::ParameterValue(0.1));
    node->get_parameter(name_ + ".check_radius", check_radius_);

    declareParameter("number_clearing_threshold", rclcpp::ParameterValue(2));
    node->get_parameter(name_ + ".number_clearing_threshold", number_clearing_threshold_);

    declareParameter("use_global_frame_to_mark", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + ".use_global_frame_to_mark", use_global_frame_to_mark_);


    marking_height_under_ground_ = 100.0;
    marking_height_above_ground_ = -100.0;

    global_frame_ = std::string(layered_costmap_->getGlobalFrameID());
    RCLCPP_INFO( logger_, "%s's global frame is %s.", getName().c_str(), global_frame_.c_str());

    double transform_tolerance;
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
    node->get_parameter(name_ + ".transform_tolerance", transform_tolerance);

    std::string topics_string;
    declareParameter("observation_sources", rclcpp::ParameterValue(std::string("")));
    node->get_parameter(name_ + ".observation_sources", topics_string);
    RCLCPP_INFO(logger_, "Subscribed to Topics: %s", topics_string.c_str());

    std::stringstream ss(topics_string);
    std::string source;

    while (ss >> source)
    {
      double observation_keep_time; 
      double expected_update_rate;
      double min_obstacle_height;
      double max_obstacle_height;
      std::string topic;
      std::string sensor_frame;
      std::string data_type;
      bool inf_is_valid=false;
      bool clearing;
      bool marking;
      double FOV_V;
      double FOV_W;
      double min_detect_distance;
      double max_detect_distance;

      declareParameter(source + "." + "topic", rclcpp::ParameterValue(std::string("")));
      node->get_parameter(name_ + "." + source + "." + "topic", topic);

      declareParameter(source + "." + "sensor_frame", rclcpp::ParameterValue(std::string("")));
      node->get_parameter(name_ + "." + source + "." + "sensor_frame", sensor_frame);

      declareParameter(source + "." + "observation_persistence", rclcpp::ParameterValue(0.0));
      node->get_parameter(name_ + "." + source + "." + "observation_persistence", observation_keep_time);

      declareParameter(source + "." + "expected_update_rate", rclcpp::ParameterValue(0.0));
      node->get_parameter(name_ + "." + source + "." + "expected_update_rate",expected_update_rate);

      declareParameter(source + "." + "min_obstacle_height", rclcpp::ParameterValue(0.0));
      node->get_parameter(name_ + "." + source + "." + "min_obstacle_height", min_obstacle_height);

      declareParameter(source + "." + "max_obstacle_height", rclcpp::ParameterValue(2.0));
      node->get_parameter(name_ + "." + source + "." + "max_obstacle_height", max_obstacle_height);

      declareParameter(source + "." + "inf_is_valid", rclcpp::ParameterValue(false));
      node->get_parameter(name_ + "." + source + "." + "inf_is_valid", inf_is_valid);

      declareParameter(source + "." + "clearing", rclcpp::ParameterValue(false));
      node->get_parameter(name_ + "." + source + "." + "clearing", clearing);

      declareParameter(source + "." + "marking", rclcpp::ParameterValue(true));
      node->get_parameter(name_ + "." + source + "." + "marking", marking);

      declareParameter(source + "." + "FOV_V", rclcpp::ParameterValue(1.0)); // radians
      node->get_parameter(name_ + "." + source + "." + "FOV_V", FOV_V);

      declareParameter(source + "." + "FOV_W", rclcpp::ParameterValue(1.5)); // radians
      node->get_parameter(name_ + "." + source + "." + "FOV_W", FOV_W);

      declareParameter(source + "." + "min_detect_distance", rclcpp::ParameterValue(0.15));
      node->get_parameter(name_ + "." + source + "." + "min_detect_distance", min_detect_distance);

      declareParameter(source + "." + "max_detect_distance", rclcpp::ParameterValue(2.5));
      node->get_parameter(name_ + "." + source + "." + "max_detect_distance", max_detect_distance);

      /// Update minimum height and maximum height for overall marking
      marking_height_above_ground_ = std::max(marking_height_above_ground_,max_obstacle_height);
      marking_height_under_ground_ = std::min(marking_height_under_ground_,min_obstacle_height);
      RCLCPP_WARN(logger_,"Update minimum height of markings to: %.2f, maximum height of markings to %.2f", 
                          marking_height_under_ground_, marking_height_above_ground_);

      std::string raytrace_range_param_name, obstacle_range_param_name;

      /// get the obstacle range for the sensor
      double obstacle_range = 2.5;
      declareParameter(source + "." + "obstacle_range", rclcpp::ParameterValue(2.5));
      node->get_parameter(name_ + "." + source + "." + "obstacle_range", obstacle_range);


      /// get the raytrace range for the sensor
      double raytrace_range = 3.0;
      declareParameter(source + "." + "raytrace_range", rclcpp::ParameterValue(3.0));
      node->get_parameter(name_ + "." + source + "." + "raytrace_range", raytrace_range);

      RCLCPP_WARN(logger_,"Creating an observation buffer for source %s, topic %s, frame %s", 
                            source.c_str(), topic.c_str(),sensor_frame.c_str());

      /// create an observation buffer
      observation_buffers_.push_back(std::shared_ptr <ObservationBufferDepth> (new ObservationBufferDepth(
        topic,
        observation_keep_time,
        expected_update_rate,
        min_obstacle_height,
        max_obstacle_height,
        obstacle_range,
        raytrace_range,
        *tf_,
        global_frame_,
        sensor_frame,
        transform_tolerance,
        FOV_V,
        FOV_W,
        min_detect_distance,
        max_detect_distance,
        node->get_clock(),
        node->get_logger())));

        /// check if we'll add this buffer to our marking observation buffers
      if (marking)
      {
        marking_buffers_.push_back(observation_buffers_.back());
      }

      /// check if we'll also add this buffer to our clearing observation buffers
      if (clearing)
      {
        clearing_buffers_.push_back(observation_buffers_.back());
      }

      RCLCPP_WARN(logger_,
        "Created an observation buffer for source %s, topic %s, global frame: %s, "
        "expected update rate: %.2f, observation persistence: %.2f",
        source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

      /// create a callback for the topic
      rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
      custom_qos_profile.depth = 10;

      auto sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2,
          rclcpp_lifecycle::LifecycleNode>>(node, topic, custom_qos_profile, sub_opt);

      if (inf_is_valid)
      {
        RCLCPP_WARN(logger_,"depth_camera_obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
      }

      std::shared_ptr < tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>
      > filter(new tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>(*sub, *tf_, global_frame_, 5,
        node->get_node_logging_interface(),
        node->get_node_clock_interface(),
        tf2::durationFromSec(0.2)));

      filter->registerCallback(
        std::bind(&DepthCameraObstacleLayer::pointCloud2Callback, this, std::placeholders::_1, observation_buffers_.back()));

      observation_subscribers_.push_back(sub);
      observation_notifiers_.push_back(filter);
    

      if (sensor_frame != "")
      {
        std::vector < std::string > target_frames;
        target_frames.reserve(2);
        target_frames.push_back(global_frame_);
        target_frames.push_back(sensor_frame);
        observation_notifiers_.back()->setTargetFrames(target_frames);
      }

    }

    enable_obstacle_layer_sub_ = node->create_subscription<std_msgs::msg::Bool>("/enable_obstacle_layer", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                            std::bind(&DepthCameraObstacleLayer::enableObstacleLayerCB, this, std::placeholders::_1));

    RCLCPP_WARN(logger_,"++++++++++++++++++++++++++++++++++++++++++++++++++++");
    RCLCPP_WARN(logger_, "%s initialization complete!", getName().c_str());
    RCLCPP_WARN(logger_,"++++++++++++++++++++++++++++++++++++++++++++++++++++");
    enabled_ = true;
    footprint_clearing_enabled_=true;
  }

  void DepthCameraObstacleLayer::enableObstacleLayerCB(const std_msgs::msg::Bool::SharedPtr msg){
    restricted_ = !msg->data;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
                                              double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (rolling_window_)
    {
      updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
    }
    
    if (!enabled_)
    {
      return;
    }

    useExtraBounds(min_x, min_y, max_x, max_y);
    
    if(!has_costmap_initial_){
      return;
    }

    //RCLCPP_INFO(logger_,"ec distance: %.2f",ec_seg_distance_);
    //RCLCPP_INFO(logger_,"ec seg min size: %.d",ec_cluster_min_size_);
    //RCLCPP_INFO(logger_,"voxel resolution: %.2f",voxel_resolution_);
    //RCLCPP_INFO(logger_,"check radius: %.2f",check_radius_);

    bool current = true;
    std::vector<ObservationDepth> observations, clearing_observations;

    /// get the marking observations
    current = current && getMarkingObservations(observations);

    /// get the clearing observations
    //current = current && getClearingObservations(clearing_observations);

    /// update the global current status
    current_ = current;

    ///publish frustum points for visualization
    pubFrustum(observations); 

    ///combine all pointcloud from all observations
    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_observations(new pcl::PointCloud<pcl::PointXYZI>);
    for (std::vector<nav2_costmap_2d::ObservationDepth>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
      const nav2_costmap_2d::ObservationDepth& obs = *it;
      *combined_observations += *(obs.cloud_);
    }

    ///Given combined pointcloud to clear the markings by kd-tree method
    ClearMarkingbyKdtree(combined_observations, observations, robot_x, robot_y, min_x, min_y, max_x, max_y);

    /// For cluster pub
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_clustered2pub(new pcl::PointCloud<pcl::PointXYZI>);
    int intensity_cnt = 100;

    if(combined_observations->points.size()>5)
    {
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
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          cloud_cluster->points.push_back (combined_observations->points[*pit]); 
          ///For visualization purpose
          pcl::PointXYZI i_pt;
          i_pt.x = combined_observations->points[*pit].x;
          i_pt.y = combined_observations->points[*pit].y;
          i_pt.z = combined_observations->points[*pit].z;
          i_pt.intensity = intensity_cnt;
          cloud_clustered2pub->push_back(i_pt);
        } 
        intensity_cnt += 100;
        if(cloud_cluster->points.size()<=(unsigned int)size_of_cluster_rejection_)
        {
          ///Do not add into markings
          continue;
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        /// Put everything in the map (pc_3d_map_global_)
        ProcessCluster(observations, cloud_cluster, min_x, min_y, max_x, max_y);
      }  
      /// Publish clustered pointcloud
      if(cluster_pub_->get_subscription_count()>0)
      {
        cluster_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_clustered2pub, *cluster_msg_);
        cluster_msg_->header.frame_id = global_frame_;
        cluster_pub_->publish(*cluster_msg_);
      }
      
    }
    
    updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  }

  ////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                                             int min_i, int min_j, int max_i, int max_j)
  {

    if (!enabled_)
    {
      return;
    }

    if(!has_costmap_initial_){

      Costmap2D * master = layered_costmap_->getCostmap();
      auto size_x = master->getSizeInCellsX();
      auto size_y = master->getSizeInCellsY();
      has_costmap_initial_ = true;
      RCLCPP_WARN(logger_.get_child(name_), "%u, %u", size_x, size_y);
      resizeMap(
        master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
        master->getOriginX(), master->getOriginY());
    }
    
    if(restricted_)
      return;

    unsigned int mx, my; 

    if(!use_global_frame_to_mark_)
    {
      for(auto it_3d_map=pc_3d_map_.begin();it_3d_map!=pc_3d_map_.end();)
      {
        indexToCells((*it_3d_map).first, mx, my);

        ///mx+1/mx-1 check is used to solve an issue of rounding problem, due to rounding may shift the index 1 step ahead
        if(isValid(mx-1,my-1) && isValid(mx-1,my+1) && isValid(mx+1,my-1) && isValid(mx+1,my+1))
        {
          unsigned int index = getIndex(mx,my);
          if((*it_3d_map).second.empty()){
            costmap_[index] = nav2_costmap_2d::FREE_SPACE;
            ++it_3d_map;
          }
          else{
            costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
            ++it_3d_map;
          }
        }
        else
        {
          pc_3d_map_.erase(it_3d_map++);
        }
      }
    }
    else
    {
      double wx, wy;
      for(auto it_3d_map=pc_3d_map_global_.begin();it_3d_map!=pc_3d_map_global_.end();)
      {
        intIndexToWorld(wx, wy, (*it_3d_map).first.first, (*it_3d_map).first.second, layered_costmap_->getCostmap()->getResolution());
        
        //RCLCPP_INFO_STREAM(logger_,"grid resolution: " << layered_costmap_->getCostmap()->getResolution() << ", (wx,wy): (" << wx << "," << wy << ")" );

        if (!worldToMap(wx, wy, mx, my))
        {
          //RCLCPP_INFO(logger_,"[updateCosts] Computing map coords failed");
          ++it_3d_map;
          continue;
        }   
        
        /// mx+1/mx-1 check is used to solve an issue of rounding problem, due to rounding may shift the index 1 step ahead
        if(isValid(mx-1,my-1) && isValid(mx-1,my+1) && isValid(mx+1,my-1) && isValid(mx+1,my+1))
        {
          unsigned int index = getIndex(mx,my);
          if((*it_3d_map).second.empty()){
            costmap_[index] = nav2_costmap_2d::FREE_SPACE;
            ++it_3d_map;
          }
          else{
            costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
            ++it_3d_map;
          }

        }
        else
        {
          pc_3d_map_global_.erase(it_3d_map++);
        }  
      }
    }

    if (footprint_clearing_enabled_)
    {
      setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE);
    }

    switch (1)
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

  //////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::activate()
  {
    RCLCPP_INFO(logger_, "%s was activated.", getName().c_str());

    /// if we're stopped we need to re-subscribe to topics
    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
      {
        observation_subscribers_[i]->subscribe();
      }
    }
    
    for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
    {
      if (observation_buffers_[i])
      {
        observation_buffers_[i]->resetLastUpdated();
      }
    }
  }
  
  /////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::deactivate()
  {
    /// unsubscribe from all sensor sources
    RCLCPP_INFO(logger_, "%s was deactivated.", getName().c_str());

    for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
    {
      if (observation_subscribers_[i] != NULL)
      {
        observation_subscribers_[i]->unsubscribe();
      }
    }
    pc_3d_map_.clear();
    pc_3d_map_global_.clear();
  }

  void DepthCameraObstacleLayer::reset(void)
  {

    RCLCPP_INFO(logger_, "%s was reseted.", getName().c_str());
    deactivate();
    resetMaps();
    current_ = true;
    activate();
  }

  /////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::pointCloud2Callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& message,
                                                     const std::shared_ptr<ObservationBufferDepth>& buffer)
  {
    /// buffer the point cloud
    buffer->lock();
    buffer->bufferCloud(*message);
    buffer->unlock();
    current_ = true;
  }

  //////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::addStaticObservation(nav2_costmap_2d::ObservationDepth& obs,
                                                      bool marking,
                                                      bool clearing)
  {
    if (marking)
    {
      static_marking_observations_.push_back(obs);
    }

    if (clearing)
    {
      static_clearing_observations_.push_back(obs);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::clearStaticObservations(bool marking, bool clearing)
  {
    if (marking)
    {
      static_marking_observations_.clear();
    }

    if (clearing)
    {
      static_clearing_observations_.clear();
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////
  bool DepthCameraObstacleLayer::getMarkingObservations(std::vector<ObservationDepth>& marking_observations) const
  {
    bool current = true;
    
    /// get the marking observations
    for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
    {
      marking_buffers_[i]->lock();
      marking_buffers_[i]->getObservations(marking_observations);
      current = marking_buffers_[i]->isCurrent() && current;
      marking_buffers_[i]->unlock();
    }
    marking_observations.insert(marking_observations.end(),
                                static_marking_observations_.begin(),
                                static_marking_observations_.end());
    return current;
  }

  ///////////////////////////////////////////////////////////////////////////////////////
  bool DepthCameraObstacleLayer::getClearingObservations(std::vector<ObservationDepth>& clearing_observations) const
  {

    bool current = true;
    /// get the clearing observations
    for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
    {
      clearing_buffers_[i]->lock();
      clearing_buffers_[i]->getObservations(clearing_observations);
      current = clearing_buffers_[i]->isCurrent() && current;
      clearing_buffers_[i]->unlock();
    }
    clearing_observations.insert(clearing_observations.end(),
                                 static_clearing_observations_.begin(),
                                 static_clearing_observations_.end());
    return current;
  }

  //////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw,
                                                 double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!footprint_clearing_enabled_)
    {
      return;
    }
    
    nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

    for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
    {
      touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::pubFrustum(std::vector<nav2_costmap_2d::ObservationDepth>& observations)
  {

    visualization_msgs::msg::MarkerArray frustums_marker_array;
    int cnt = 0;

    for (std::vector<nav2_costmap_2d::ObservationDepth>::const_iterator it = observations.begin(); it != observations.end(); ++it)
    {
      visualization_msgs::msg::Marker frustum_marker;
      frustum_marker.header.frame_id = global_frame_;
      frustum_marker.action = visualization_msgs::msg::Marker::ADD;
      frustum_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      frustum_marker.pose.orientation.w = 1.0;
      frustum_marker.ns = std::to_string(cnt);
      frustum_marker.id = 0;
      const nav2_costmap_2d::ObservationDepth& obs = *it;
      /* Note of frustum sequence -> TLNear: Top Left Near
      TLNear = frustum_->points[0];
      TRNear = frustum_->points[1];
      BLNear = frustum_->points[2];
      BRNear = frustum_->points[3];
      TLFar = frustum_->points[4];
      TRFar = frustum_->points[5];
      BLFar = frustum_->points[6];
      BRFar = frustum_->points[7];
      */

      frustum_marker.scale.x = 0.02;
      frustum_marker.color.r = 0.1; frustum_marker.color.g = 0.1; frustum_marker.color.b = 0.7;
      frustum_marker.color.a = 0.5;
      geometry_msgs::msg::Point gpt1, gpt2;
      gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
      gpt2.x = obs.frustum_->points[1].x; gpt2.y = obs.frustum_->points[1].y; gpt2.z = obs.frustum_->points[1].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[2].x; gpt1.y = obs.frustum_->points[2].y; gpt1.z = obs.frustum_->points[2].z; 
      gpt2.x = obs.frustum_->points[3].x; gpt2.y = obs.frustum_->points[3].y; gpt2.z = obs.frustum_->points[3].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[4].x; gpt1.y = obs.frustum_->points[4].y; gpt1.z = obs.frustum_->points[4].z; 
      gpt2.x = obs.frustum_->points[5].x; gpt2.y = obs.frustum_->points[5].y; gpt2.z = obs.frustum_->points[5].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[6].x; gpt1.y = obs.frustum_->points[6].y; gpt1.z = obs.frustum_->points[6].z; 
      gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
      gpt2.x = obs.frustum_->points[4].x; gpt2.y = obs.frustum_->points[4].y; gpt2.z = obs.frustum_->points[4].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[1].x; gpt1.y = obs.frustum_->points[1].y; gpt1.z = obs.frustum_->points[1].z; 
      gpt2.x = obs.frustum_->points[5].x; gpt2.y = obs.frustum_->points[5].y; gpt2.z = obs.frustum_->points[5].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[2].x; gpt1.y = obs.frustum_->points[2].y; gpt1.z = obs.frustum_->points[2].z; 
      gpt2.x = obs.frustum_->points[6].x; gpt2.y = obs.frustum_->points[6].y; gpt2.z = obs.frustum_->points[6].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[3].x; gpt1.y = obs.frustum_->points[3].y; gpt1.z = obs.frustum_->points[3].z; 
      gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[4].x; gpt1.y = obs.frustum_->points[4].y; gpt1.z = obs.frustum_->points[4].z; 
      gpt2.x = obs.frustum_->points[6].x; gpt2.y = obs.frustum_->points[6].y; gpt2.z = obs.frustum_->points[6].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[5].x; gpt1.y = obs.frustum_->points[5].y; gpt1.z = obs.frustum_->points[5].z; 
      gpt2.x = obs.frustum_->points[7].x; gpt2.y = obs.frustum_->points[7].y; gpt2.z = obs.frustum_->points[7].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[0].x; gpt1.y = obs.frustum_->points[0].y; gpt1.z = obs.frustum_->points[0].z; 
      gpt2.x = obs.frustum_->points[2].x; gpt2.y = obs.frustum_->points[2].y; gpt2.z = obs.frustum_->points[2].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      gpt1.x = obs.frustum_->points[1].x; gpt1.y = obs.frustum_->points[1].y; gpt1.z = obs.frustum_->points[1].z; 
      gpt2.x = obs.frustum_->points[3].x; gpt2.y = obs.frustum_->points[3].y; gpt2.z = obs.frustum_->points[3].z; 
      frustum_marker.points.push_back(gpt1);
      frustum_marker.points.push_back(gpt2);
      frustums_marker_array.markers.push_back(frustum_marker);
      cnt++;

    }
    frustum_pub_->publish(frustums_marker_array);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::ClearMarkingbyKdtree(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in, 
                                                      std::vector<nav2_costmap_2d::ObservationDepth>& observations,
                                                      double robot_x, double robot_y,
                                                      double* min_x, double* min_y, double* max_x, double* max_y)
  {
    
    pcl::PointXYZI searchPoint;

    pcl::PointCloud<pcl::PointXYZI>::Ptr marking(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_gbl;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
  
    bool bypass_clearing = false;
    if(enable_near_blocked_protection_)
    {
      if(cloud_in->points.size()<(unsigned int)number_points_considered_as_blocked_)
      {
        /// ToDo: confirm that the clk works!!
        //auto& clk = *this->get_clock();
        //RCLCPP_WARN_THROTTLE(logger_,clk, 30, "Blocked by something, clearing mechanism is skipped.");
        RCLCPP_WARN(logger_,"Blocked by something, clearing mechanism is skipped.");
        bypass_clearing = true;
      }
    }

    bool clear_all_marking_in_this_frame = false;
    if(cloud_in->points.size()>5)
    {
      kdtree_gbl.setInputCloud (cloud_in);    
    }
    else
    {
      if(!enable_near_blocked_protection_)
      {
        clear_all_marking_in_this_frame = true; //We really see nothing in the space
      }
      else
      {
        bypass_clearing = true;
      }
    }

    //RCLCPP_WARN_STREAM(logger_,"Global map points size: " << pc_3d_map_global_.size());

    
    bool is_marking_sub = marking_pub_->get_subscription_count()>0;

    ///Prepare for frustum_utils
    FrustumUtils frustum_utils(&observations);
  

    if(!use_global_frame_to_mark_)
    {
      unsigned int mx, my;
      double wx,wy;
      ///Iterate marking
      for(auto it_3d_map=pc_3d_map_.begin();it_3d_map!=pc_3d_map_.end();it_3d_map++)
      {
        indexToCells((*it_3d_map).first, mx, my);
        mapToWorld(mx, my, wx, wy);

        bool is_in_FRUSTUM = false;
        bool is_attach_FRUSTUM = false;
        /// if marked point cloud is n meters from robot, we just skip, because it is out of our frustum 
        if(hypot(wx-robot_x,wy-robot_y)>10.0 && !is_marking_sub)
        {
          RCLCPP_DEBUG(logger_,"Marked pointcloud is greater 10m and therefore out of frustum.");
          continue;
        }

        for(auto it = (*it_3d_map).second.begin(), next_it = it; it != (*it_3d_map).second.end(); it = next_it) 
        {
          ++next_it;
          searchPoint.x = wx;
          searchPoint.y = wy;
          searchPoint.z = (*it).first*voxel_resolution_;
          searchPoint.intensity = (*it).second;

          //RCLCPP_WARN(logger_,"[Clearing: before check] x,y,z: %f,%f,%f ",searchPoint.x,searchPoint.y,searchPoint.z);
          
          if(is_marking_sub)
          {
            marking->push_back(searchPoint);
          }

          pointIdxRadiusSearch.clear();
          pointRadiusSquaredDistance.clear();
          double pc_dis = hypot(wx-robot_x,wy-robot_y);
          is_in_FRUSTUM = frustum_utils.isInsideFRUSTUMs(searchPoint);
          is_attach_FRUSTUM = frustum_utils.isAttachFRUSTUMs(searchPoint);
          
          #if(DEBUG)
          RCLCPP_WARN(logger_,"[Clearing: before check] x,y,z: %f,%f,%f ----- %d, %d",searchPoint.x,searchPoint.y,searchPoint.z, is_in_FRUSTUM, is_attach_FRUSTUM);
          
          for(int k=0;k<distance_to_plane.size(); k++)
          {
            RCLCPP_WARN(logger_,"plane: %d, distance: %f, hypot: %f", k, distance_to_plane[k].first, distance_to_plane[k].second);
          }
          RCLCPP_WARN(logger_,"=============");
          #endif


          if(is_in_FRUSTUM && clear_all_marking_in_this_frame)
          {
            ///Nothing is detected, clear all markings.
            //RCLCPP_WARN(logger_,"Clear all markinging in this frame.");
            (*it_3d_map).second.erase(it);
            touch(wx, wy, min_x, min_y, max_x, max_y);
          }
          else if(is_in_FRUSTUM && !bypass_clearing && pc_dis<=forced_clearing_distance_)
          {
            //RCLCPP_WARN(logger_,"CFOV Erase : %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
            //if(pc_dis<=forced_clearing_distance_)
            //  RCLCPP_WARN(logger_,"Clear by Footprint: %.2f",pc_dis);
            (*it_3d_map).second.erase(it);
            touch(wx, wy, min_x, min_y, max_x, max_y);
          }
          else if (is_in_FRUSTUM && !bypass_clearing) 
          {
            if(kdtree_gbl.radiusSearch (searchPoint, check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance)<number_clearing_threshold_)
            {
              //RCLCPP_WARN(logger_,"Erase by kdtree: %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
              (*it_3d_map).second.erase(it);
              touch(wx, wy, min_x, min_y, max_x, max_y);
            }
          } 
          else if (!is_in_FRUSTUM && is_attach_FRUSTUM)
          {
            //(*it_3d_map).second.erase(it);
          }
          else 
          {
            //RCLCPP_WARN(logger_,"Pass: %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
            /// Should never reached here !!
            // RCLCPP_WARN(logger_,"Pass: %f,%f,%f, is_inside: %d, is_attach: %d", searchPoint.x, searchPoint.y, searchPoint.z, is_in_FRUSTUM, is_attach_FRUSTUM);
            // for(int k=0;k<distance_to_plane.size(); k++)
            // {
            //   RCLCPP_WARN(logger_,"plane: %d, distance: %f, hypot: %f", k, distance_to_plane[k].first, distance_to_plane[k].second);
            // }
            // RCLCPP_WARN(logger_,"=============");
          }
        }
      }
    }
    else
    {
      double wx,wy;
      ///Iterate marking in global frame marking mode
      for(auto it_3d_map=pc_3d_map_global_.begin();it_3d_map!=pc_3d_map_global_.end();it_3d_map++)
      {  
        intIndexToWorld(wx, wy, (*it_3d_map).first.first, (*it_3d_map).first.second, layered_costmap_->getCostmap()->getResolution());
        bool is_in_FRUSTUM = false;
        bool is_attach_FRUSTUM = false;
        
        ///if marked point cloud is n meters from robot, we just skip, because it is out of our frustum 
        if(hypot(wx-robot_x,wy-robot_y)>10.0 && !is_marking_sub)
        {
          RCLCPP_DEBUG(logger_,"Marked pointcloud is greater 10m and therefore out of frustum.");
          continue;
        }

        for(auto it = (*it_3d_map).second.begin(), next_it = it; it != (*it_3d_map).second.end(); it = next_it) 
        {
          ++next_it;

          searchPoint.x = wx;
          searchPoint.y = wy;
          searchPoint.z = (*it).first*voxel_resolution_;
          searchPoint.intensity = (*it).second;
          
          //RCLCPP_WARN(logger_,"+[Clearing: before check] x,y,z: %f,%f,%f ",searchPoint.x,searchPoint.y,searchPoint.z);

          if(is_marking_sub)
          {
            marking->push_back(searchPoint);
          }

          pointIdxRadiusSearch.clear();
          pointRadiusSquaredDistance.clear();
          double pc_dis = hypot(wx-robot_x,wy-robot_y);
          is_in_FRUSTUM = frustum_utils.isInsideFRUSTUMs(searchPoint);
          is_attach_FRUSTUM = frustum_utils.isAttachFRUSTUMs(searchPoint);

          if(is_in_FRUSTUM && clear_all_marking_in_this_frame)
          {
            ///Nothing is detected, clear all markings.
            //RCLCPP_WARN(logger_,"Clear all markinging in this frame.");
            (*it_3d_map).second.erase(it);
            touch(wx, wy, min_x, min_y, max_x, max_y);
          }
          else if(is_in_FRUSTUM && !bypass_clearing && pc_dis<=forced_clearing_distance_)
          {
            //RCLCPP_WARN(logger_,"CFOV Erase : %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
            //if(pc_dis<=forced_clearing_distance_)
            //  RCLCPP_WARN(logger_,"Clear by Footprint: %.2f",pc_dis);
            (*it_3d_map).second.erase(it);
            touch(wx, wy, min_x, min_y, max_x, max_y);
          }
          else if (is_in_FRUSTUM && !bypass_clearing) 
          {
            if(kdtree_gbl.radiusSearch (searchPoint, check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance)<number_clearing_threshold_)
            {
              //RCLCPP_WARN(logger_,"Erase by kdtree: %.2f, %.2f, %.2f", searchPoint.x, searchPoint.y, searchPoint.z);
              (*it_3d_map).second.erase(it);
              touch(wx, wy, min_x, min_y, max_x, max_y);
            }
          }
          else if (!is_in_FRUSTUM && is_attach_FRUSTUM)
          {
            //(*it_3d_map).second.erase(it);
          }
          else 
          {
            /// Should never reached here !!! 
            //RCLCPP_WARN(logger_,"+Pass: %f,%f,%f, is_inside: %d, is_attach: %d", searchPoint.x, searchPoint.y, searchPoint.z, is_in_FRUSTUM, is_attach_FRUSTUM);
            // for(int k=0;k<distance_to_plane.size(); k++)
            // {
            //   RCLCPP_WARN(logger_,"plane: %d, distance: %f, hypot:%f", k, distance_to_plane[k].first, distance_to_plane[k].second);
            // }
            // RCLCPP_WARN(logger_,"=============");
          }
        }
      }    
    }
    if(is_marking_sub)
    {
      marking_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pcl::toROSMsg(*marking, *marking_msg_);
      marking_msg_->header.frame_id = global_frame_;
      marking_pub_->publish(*marking_msg_);
    }
  }

  /////////////////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::ProcessCluster(std::vector<nav2_costmap_2d::ObservationDepth>& observations,
                                                pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster_cloud, 
                                                double* min_x, double* min_y, double* max_x, double* max_y)
  {
    geometry_msgs::msg::Point pt;

    unsigned int mx, my;
    double wx,wy;
    int mmx, mmy;
  
    ///Prepare for frustum_utils
    FrustumUtils frustum_utils(&observations); 
    
    for(unsigned int i=0;i<cluster_cloud->points.size();i++)
    {
      //RCLCPP_WARN_STREAM(logger_, "Test point (x,y,z): " << tmp_cluster_cloud->points[i].x << ","<<tmp_cluster_cloud->points[i].y <<","<<tmp_cluster_cloud->points[i].z);
  
      pt.x = cluster_cloud->points[i].x;
      pt.y = cluster_cloud->points[i].y;
      pt.z = cluster_cloud->points[i].z;
      //tf_->transformPoint (global_frame_, pt, tf_pt);

      wx = pt.x;
      wy = pt.y;

      pcl::PointXYZI searchPoint;
      searchPoint.x = wx;
      searchPoint.y = wy;
      searchPoint.z = cluster_cloud->points[i].z;

      bool is_in_FRUSTUM = frustum_utils.isInsideFRUSTUMs(searchPoint);
      bool is_attach_FRUSTUM = frustum_utils.isAttachFRUSTUMs(searchPoint);
      
      // for(unsigned int k=0; i<distance_to_plane.size(); k++)
      // {
      //   if(distance_to_plane[k].second>5.5)
      //   {
      //     continue;
      //   }
      // }
      ///These are robust marking conditions, attachment testing usually causing boundary condition.*/
      if(!is_in_FRUSTUM || is_attach_FRUSTUM )
      {  
        //RCLCPP_WARN_STREAM(logger_, "in Frustum: "<< is_in_FRUSTUM << ", is attach frustum: " << is_attach_FRUSTUM);
        continue;
      }  
    
      if(!use_global_frame_to_mark_)
      {
        if (!worldToMap(wx, wy, mx, my))
        {
          RCLCPP_DEBUG(logger_,"[ProcessCluster] Computing map coords failed");
          continue;
        }    


        unsigned int index = getIndex(mx,my);

        int h_ind = (int)round(cluster_cloud->points[i].z*(1/voxel_resolution_));
        
        if(h_ind>(int)marking_height_above_ground_/voxel_resolution_ || h_ind<(int)marking_height_under_ground_/voxel_resolution_)
        {
          continue;
        }

        insert_ptr_ = pc_3d_map_[index].insert(std::pair<int, float>(h_ind, cluster_cloud->points[i].intensity));
        
        #if(DEBUG)
        RCLCPP_WARN_STREAM(logger_,"[After insert] x,y,z: "<< searchPoint.x << "," << searchPoint.y 
                                                           << "," << searchPoint.z 
                                                           << ", is_inside: " << is_in_FRUSTUM
                                                           << ", is_attach: " << is_attach_FRUSTUM);
        #endif
        
        if(!insert_ptr_.second)//the key is already in map, put max label in it!
        {
          pc_3d_map_[index][h_ind] = std::max(pc_3d_map_[index][h_ind], cluster_cloud->points[i].intensity);
        }
        touch(wx, wy, min_x, min_y, max_x, max_y);
      }
      else
      {
        /// We use int type to store the point clouds in pc_3d_map_global_
        worldToIntIndex(wx, wy, mmx, mmy, layered_costmap_->getCostmap()->getResolution());

        /// check height which should be the same as what we did
        int h_ind = (int)round(cluster_cloud->points[i].z*(1/voxel_resolution_));
        if(h_ind>(int)marking_height_above_ground_/voxel_resolution_ || h_ind<(int)marking_height_under_ground_/voxel_resolution_)
        {
          continue;
        }
        
        ///Add into std::map
        insert_ptr_ = pc_3d_map_global_[std::pair<int, int>(mmx, mmy)].insert(std::pair<int, float>(h_ind, cluster_cloud->points[i].intensity));
        
        #if(DEBUG)  
        RCLCPP_WARN_STREAM(logger_,"[After insert] x,y,z: "<< searchPoint.x << "," << searchPoint.y 
                                                           << "," << searchPoint.z 
                                                           << ", is_inside: " << is_in_FRUSTUM
                                                           << ", is_attach: " << is_attach_FRUSTUM);

        #endif

        if(!insert_ptr_.second)//the key is already in map, put max label in it!
        {
          pc_3d_map_global_[std::pair<int, int>(mmx, mmy)][h_ind] = std::max(pc_3d_map_global_[std::pair<int, int>(mmx, mmy)][h_ind],cluster_cloud->points[i].intensity);
        }
        touch(wx, wy, min_x, min_y, max_x, max_y);
      
      }
    }
    //RCLCPP_WARN_STREAM(logger_, "cluster cloud size: " << cluster_cloud->points.size() << ", dummy count: " << conversion_error_cnt);
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  bool DepthCameraObstacleLayer::isValid(unsigned int mx, unsigned int my)
  {
    if(mx>getSizeInCellsX() || my>getSizeInCellsY())
    {
      return false;
    }
    return true;
  }

  /////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::worldToIntIndex(double wx, double wy, int& mx, int& my, double resolution) const
  {
    mx = (int)((wx) / resolution);
    my = (int)((wy) / resolution);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  void DepthCameraObstacleLayer::intIndexToWorld(double& wx, double& wy, int mx, int my, double resolution) const
  {
    wx = mx * resolution;
    wy = my * resolution;
  }

}  // namespace nav2_costmap_2d

// Register the macro for this layer
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DepthCameraObstacleLayer, nav2_costmap_2d::Layer)