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
#include <costmap_depth_camera/observation_buffer.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

/*
pcl::removeNaNFromPointCloud
*/
#include <pcl/filters/filter.h>

/*
pcl_ros::transformPointCloud
*/
#include <pcl_ros/transforms.h>

using namespace std;
using namespace tf2;

namespace costmap_depth_camera
{
ObservationBuffer::ObservationBuffer(string topic_name, double observation_keep_time, double expected_update_rate,
                                     double min_obstacle_height, double max_obstacle_height, double obstacle_range,
                                     double raytrace_range, tf2_ros::Buffer& tf2_buffer, string global_frame,
                                     string sensor_frame, double tf_tolerance,
                                     double FOV_V, double FOV_W, double min_detect_distance, double max_detect_distance) :
    tf2_buffer_(tf2_buffer), observation_keep_time_(observation_keep_time), expected_update_rate_(expected_update_rate),
    last_updated_(ros::Time::now()), global_frame_(global_frame), sensor_frame_(sensor_frame), topic_name_(topic_name),
    min_obstacle_height_(min_obstacle_height), max_obstacle_height_(max_obstacle_height),
    obstacle_range_(obstacle_range), raytrace_range_(raytrace_range), tf_tolerance_(tf_tolerance),
    FOV_V_(FOV_V), FOV_W_(FOV_W), min_detect_distance_(min_detect_distance), max_detect_distance_(max_detect_distance)
{
}

ObservationBuffer::~ObservationBuffer()
{
}

bool ObservationBuffer::setGlobalFrame(const std::string new_global_frame)
{
  ros::Time transform_time = ros::Time::now();
  std::string tf_error;

  geometry_msgs::TransformStamped transformStamped;
  if (!tf2_buffer_.canTransform(new_global_frame, global_frame_, transform_time, ros::Duration(tf_tolerance_), &tf_error))
  {
    ROS_ERROR("Transform between %s and %s with tolerance %.2f failed: %s.", new_global_frame.c_str(),
              global_frame_.c_str(), tf_tolerance_, tf_error.c_str());
    return false;
  }

  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    try
    {
      Observation& obs = *obs_it;

      geometry_msgs::PointStamped origin;
      origin.header.frame_id = global_frame_;
      origin.header.stamp = transform_time;
      origin.point = obs.origin_;

      // we need to transform the origin of the observation to the new global frame
      tf2_buffer_.transform(origin, origin, new_global_frame);
      obs.origin_ = origin.point;

      // we also need to transform the cloud of the observation to the new global frame
      tf2_buffer_.transform(*(obs.cloud_), *(obs.cloud_), new_global_frame);
    }
    catch (TransformException& ex)
    {
      ROS_ERROR("TF Error attempting to transform an observation from %s to %s: %s", global_frame_.c_str(),
                new_global_frame.c_str(), ex.what());
      return false;
    }
  }

  // now we need to update our global_frame member
  global_frame_ = new_global_frame;
  return true;
}

void ObservationBuffer::bufferCloud(const sensor_msgs::PointCloud2& cloud)
{
  geometry_msgs::PointStamped global_origin;

  // create a new observation on the list to be populated
  observation_list_.push_front(Observation());

  // check whether the origin frame has been set explicitly or whether we should get it from the cloud
  string origin_frame = sensor_frame_ == "" ? cloud.header.frame_id : sensor_frame_;

  try
  {
    // given these observations come from sensors... we'll need to store the origin pt of the sensor
    geometry_msgs::PointStamped local_origin;
    local_origin.header.stamp = cloud.header.stamp;
    local_origin.header.frame_id = origin_frame;
    local_origin.point.x = 0;
    local_origin.point.y = 0;
    local_origin.point.z = 0;
    tf2_buffer_.transform(local_origin, global_origin, global_frame_);
    tf2::convert(global_origin.point, observation_list_.front().origin_);
    
    /*
    Update camera parameters
    */
    observation_list_.front().min_detect_distance_ = min_detect_distance_;
    observation_list_.front().max_detect_distance_ = max_detect_distance_;
    observation_list_.front().FOV_W_ = FOV_W_;
    observation_list_.front().FOV_V_ = FOV_V_;
    
    /*
    Find frustum vertex (8 points) and transform it to global.
    !!! Frustum vertex is usually based on camera_link frame (realsense).
    */
    observation_list_.front().findFrustumVertex();
    pcl_conversions::toPCL(cloud.header.stamp, observation_list_.front().frustum_->header.stamp);
    observation_list_.front().frustum_->header.frame_id = origin_frame;
    pcl_ros::transformPointCloud(global_frame_, *observation_list_.front().frustum_, *observation_list_.front().frustum_, tf2_buffer_);
    observation_list_.front().frustum_->header.frame_id = global_frame_;
    
    /*
    Find frustum normal and plane, note that the planes/normals are in global frame
    !!! findFrustumNormal() will assign BRNear_&&TLFar_  which are both in global frame
    */
    observation_list_.front().findFrustumNormal();
    observation_list_.front().findFrustumPlane();
    
    // transform the point cloud, from camera_depth_optical_frame
    sensor_msgs::PointCloud2 global_frame_cloud;
    tf2_buffer_.transform(cloud, global_frame_cloud, global_frame_);
    global_frame_cloud.header.stamp = cloud.header.stamp;

    // copy over the points that are within our height bounds
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(global_frame_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(global_frame_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(global_frame_cloud, "z");
    
    for (; iter_x !=iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if ((*iter_z) <= max_obstacle_height_
          && (*iter_z) >= min_obstacle_height_)
      {
        pcl::PointXYZI tmp_pt;
        tmp_pt.x = *iter_x;
        tmp_pt.y = *iter_y;
        tmp_pt.z = *iter_z;
        tmp_pt.intensity = 0;
        observation_list_.front().cloud_->push_back(tmp_pt);
      }
    }

    pcl_conversions::toPCL(ros::Time::now(), observation_list_.front().cloud_->header.stamp);
    observation_list_.front().cloud_->header.frame_id = global_frame_;

  }
  catch (TransformException& ex)
  {
    // if an exception occurs, we need to remove the empty observation from the list
    observation_list_.pop_front();
    ROS_ERROR("TF Exception that should never happen for sensor frame: %s, cloud frame: %s, %s", sensor_frame_.c_str(),
              cloud.header.frame_id.c_str(), ex.what());
    return;
  }

  // if the update was successful, we want to update the last updated time
  last_updated_ = ros::Time::now();

  // we'll also remove any stale observations from the list
  purgeStaleObservations();
}

// returns a copy of the observations
void ObservationBuffer::getObservations(vector<Observation>& observations)
{
  // first... let's make sure that we don't have any stale observations
  purgeStaleObservations();

  // now we'll just copy the observations for the caller
  list<Observation>::iterator obs_it;
  for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
  {
    observations.push_back(*obs_it);
  }
}

void ObservationBuffer::purgeStaleObservations()
{
  if (!observation_list_.empty())
  {
    list<Observation>::iterator obs_it = observation_list_.begin();
    // if we're keeping observations for no time... then we'll only keep one observation
    if (observation_keep_time_ == ros::Duration(0.0))
    {
      observation_list_.erase(++obs_it, observation_list_.end());
      return;
    }

    // otherwise... we'll have to loop through the observations to see which ones are stale
    for (obs_it = observation_list_.begin(); obs_it != observation_list_.end(); ++obs_it)
    {
      costmap_depth_camera::Observation& obs = *obs_it;
      ros::Duration time_diff = last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp;
      if ((last_updated_ - pcl_conversions::fromPCL(obs.cloud_->header).stamp) > observation_keep_time_)
      {
        observation_list_.erase(obs_it, observation_list_.end());
        return;
      }
    }
  }
}

bool ObservationBuffer::isCurrent() const
{
  if (expected_update_rate_ == ros::Duration(0.0))
    return true;

  bool current = (ros::Time::now() - last_updated_).toSec() <= expected_update_rate_.toSec();
  if (!current)
  {
    ROS_WARN(
        "The %s observation buffer has not been updated for %.2f seconds, and it should be updated every %.2f seconds.",
        topic_name_.c_str(), (ros::Time::now() - last_updated_).toSec(), expected_update_rate_.toSec());
  }
  return current;
}

void ObservationBuffer::resetLastUpdated()
{
  last_updated_ = ros::Time::now();
}
}  // namespace costmap_depth_camera

