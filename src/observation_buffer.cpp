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
#include <rclcpp/rclcpp.hpp>
#include <costmap_depth_camera/observation_buffer.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.hpp>

using namespace std;
using namespace tf2;

namespace nav2_costmap_2d
{
  using namespace std::chrono_literals;

ObservationBufferDepth::ObservationBufferDepth(std::string topic_name,
                                     double observation_keep_time,
                                     double expected_update_rate,
                                     double min_obstacle_height,
                                     double max_obstacle_height,
                                     double obstacle_range,
                                     double raytrace_range,
                                     tf2_ros::Buffer& tf2_buffer,
                                     std::string global_frame,
                                     std::string sensor_frame,
                                     double tf_tolerance,
                                     double FOV_V,
                                     double FOV_W,
                                     double min_detect_distance,
                                     double max_detect_distance,
                                     bool use_voxelized_observation,
                                     rclcpp::Clock::SharedPtr clock,
                                     rclcpp::Logger logger) 
: tf2_buffer_(tf2_buffer)
, observation_keep_time_(rclcpp::Duration::from_seconds(observation_keep_time))
, expected_update_rate_(rclcpp::Duration::from_seconds(expected_update_rate))
, last_updated_(clock->now())
, global_frame_(global_frame)
, sensor_frame_(sensor_frame)
, topic_name_(topic_name)
, min_obstacle_height_(min_obstacle_height)
, max_obstacle_height_(max_obstacle_height)
, obstacle_range_(obstacle_range)
, raytrace_range_(raytrace_range)
, tf_tolerance_(tf_tolerance)
, FOV_V_(FOV_V)
, FOV_W_(FOV_W)
, min_detect_distance_(min_detect_distance)
, max_detect_distance_(max_detect_distance)
, use_voxelized_observation_(use_voxelized_observation)
, clock_(clock)
, logger_(logger)
{
}

ObservationBufferDepth::~ObservationBufferDepth()
{
}

void ObservationBufferDepth::bufferCloud(const sensor_msgs::msg::PointCloud2& cloud)
{
  
  observation_.cloud_->points.clear();
  observation_.frustum_->points.clear();
  observation_.frustum_normal_->points.clear();
  observation_.frustum_plane_equation_.clear();
  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;
  
  if(sensor_frame_ == "")
  {
    RCLCPP_WARN_STREAM(logger_,"Warning: Sensor frame is not provided in yaml file. Using pointcloud header frame id");
  }
  
  /// Check the cloud size 
  pcl::PointCloud<pcl::PointXYZI>::Ptr rawcloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloud, *rawcloud);
  
  //voxelized pc to save computation
  if(use_voxelized_observation_){
    pcl::VoxelGrid<pcl::PointXYZI> ds_rawcloud;
    ds_rawcloud.setLeafSize(0.05, 0.05, 0.05);
    ds_rawcloud.setInputCloud(rawcloud);
    ds_rawcloud.filter(*rawcloud);
  }

  if(rawcloud->size() >20000)
  {
    RCLCPP_ERROR_STREAM(logger_, "Raw cloud size " << rawcloud->size() <<" is larger than 20000 points. Exiting.. ");
    return;
  }
  
  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::msg::TransformStamped T_S_C_msg;
    try{
      T_S_C_msg = tf2_buffer_.lookupTransform(global_frame_, origin_frame, tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_DEBUG(logger_, "%s", e.what());
      return;
    }

    observation_.origin_.x = T_S_C_msg.transform.translation.x;
    observation_.origin_.y = T_S_C_msg.transform.translation.y;
    observation_.origin_.z = T_S_C_msg.transform.translation.z;
    
    /// Update camera parameters
    observation_.min_detect_distance_ = min_detect_distance_;
    observation_.max_detect_distance_ = max_detect_distance_;
    observation_.FOV_W_ = FOV_W_;
    observation_.FOV_V_ = FOV_V_;
    
    /// Find frustum vertex (8 points) and transform it to global.
    /// !!! Frustum vertex is usually based on camera_link frame (realsense).
    observation_.findFrustumVertex();
    
    pcl_conversions::toPCL(cloud.header.stamp, observation_.frustum_->header.stamp);
    observation_.frustum_->header.frame_id = origin_frame;

    Eigen::Affine3d trans_m2s_af3 = tf2::transformToEigen(T_S_C_msg);
    pcl::transformPointCloud(*observation_.frustum_, *observation_.frustum_, trans_m2s_af3);
    
    observation_.frustum_->header.frame_id = global_frame_;
    
    /// Find frustum normal and plane, note that the planes/normals are in global frame
    /// !!! findFrustumNormal() will assign BRNear_&&TLFar_  which are both in global frame
    observation_.findFrustumNormal();
    observation_.findFrustumPlane();
    /// Transform the point cloud to global frame (basically z pointing up), from sensor frame
    geometry_msgs::msg::TransformStamped ros_tf_global2optical;

    try{
      ros_tf_global2optical = tf2_buffer_.lookupTransform(global_frame_, cloud.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_DEBUG(logger_, "%s", e.what());
      return;
    }

    Eigen::Affine3d trans_m2optical_af3 = tf2::transformToEigen(ros_tf_global2optical);
    pcl::transformPointCloud(*rawcloud, *rawcloud, trans_m2optical_af3);
    
    for(auto rit=rawcloud->points.begin(); rit!=rawcloud->points.end(); rit++){
      if ((*rit).z <= max_obstacle_height_ && (*rit).z >= min_obstacle_height_)
      {
        observation_.cloud_->push_back((*rit));
      }
    }
    
    if(observation_.cloud_->size() >10000)
    {
      RCLCPP_ERROR_STREAM(logger_, "ObservationDepth size " << observation_.cloud_->size() <<" is larger than 10000 points. Exiting.. ");
      return;
    }

    pcl_conversions::toPCL(clock_->now(), observation_.cloud_->header.stamp);
    observation_.cloud_->header.frame_id = global_frame_;

    ///RCLCPP_WARN_STREAM(logger_, "++++++observation cloud size: " << observation_.cloud_->size() << ", count: " << tmp_count);
  }
  catch (TransformException& ex)
  {
    RCLCPP_ERROR(logger_,"TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
                 cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = clock_->now();

}

// returns a copy of the observation
void ObservationBufferDepth::getObservations(nav2_costmap_2d::ObservationDepth& observation)
{
  //do copy instead of ptr copy
  (*observation.cloud_) = (*observation_.cloud_);
  (*observation.frustum_) = (*observation_.frustum_);
  (*observation.frustum_normal_) = (*observation_.frustum_normal_);
  observation.frustum_plane_equation_ = observation_.frustum_plane_equation_;
  observation.FOV_V_ = observation_.FOV_V_;
  observation.FOV_W_ = observation_.FOV_W_;
  observation.min_detect_distance_ = observation_.min_detect_distance_;
  observation.max_detect_distance_ = observation_.max_detect_distance_;
  observation.BRNear_ = observation_.BRNear_;
  observation.TLFar_ = observation_.TLFar_;
}

bool ObservationBufferDepth::isCurrent() const
{ 
  if (expected_update_rate_ == rclcpp::Duration(rclcpp::Duration::from_seconds(0.0)))
    return true;

  const rclcpp::Duration update_time = clock_->now() - last_updated_;
  bool current = update_time.seconds() <= expected_update_rate_.seconds();
  
  if (!current)
  {
    RCLCPP_WARN(logger_, "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
      topic_name_.c_str(), update_time.seconds(), expected_update_rate_.seconds());
  }
  return current;
}

void ObservationBufferDepth::resetLastUpdated()
{

  last_updated_ = clock_->now();
}

}  // namespace nav2_costmap_2d

