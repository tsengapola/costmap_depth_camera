/*
 * Copyright (c) 2008, 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Apola
 */

#ifndef COSTMAP_DEPTH_CAMERA_OBSERVATION_H_
#define COSTMAP_DEPTH_CAMERA_OBSERVATION_H_

#include <geometry_msgs/msg/point.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace costmap_depth_camera
{

/**
 * @brief Stores an observation in terms of a point cloud and the origin of the source
 * @note Tried to make members and constructor arguments const but the compiler would not accept the default
 * assignment operator for vector insertion!
 */
class Observation
{
public:
  /**
   * @brief  Creates an empty observation
   */
  Observation();

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation& obs);

  virtual ~Observation();


  pcl::PointXYZ getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  pcl::PointXYZ getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  void getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);
  void findFrustumVertex();
  void findFrustumNormal();
  void findFrustumPlane();


  /// These points are for frustum check
  geometry_msgs::msg::Point origin_;
  pcl::PointCloud<pcl::PointXYZI>* cloud_;
  pcl::PointCloud<pcl::PointXYZ>* frustum_;
  pcl::PointCloud<pcl::PointXYZ>* frustum_normal_;
  std::vector<Eigen::Vector4f> frustum_plane_equation_;

  /// These parameter is essential for depth camera
  double FOV_V_;
  double FOV_W_;
  double min_detect_distance_;
  double max_detect_distance_;

  pcl::PointXYZ BRNear_;
  pcl::PointXYZ TLFar_;
  

}; /// class Observation
}  /// namespace costmap_depth_camera
#endif  // COSTMAP_DEPTH_CAMERA_OBSERVATION_H_

