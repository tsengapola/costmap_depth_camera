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
#ifndef COSTMAP_DEPTH_CAMERA_FRUSTUM_UTILS_H_
#define COSTMAP_DEPTH_CAMERA_FRUSTUM_UTILS_H_

#include <costmap_depth_camera/observation_buffer.h>
namespace costmap_depth_camera
{
  class FrustumUtils
  {
    public:

      FrustumUtils(std::vector<costmap_depth_camera::Observation>* observations);
      ~FrustumUtils();

      /*
      Test if a point is attached frustums (for boundary test)
      */
      bool isAttachFRUSTUMs(pcl::PointXYZI testPoint, double& distance);

      /*
      This function is used to check a point attached on one frustum but inside another frustum
      */
      bool isInsideFRUSTUMwoAttach(costmap_depth_camera::Observation& observation, pcl::PointXYZI testPoint);

      /*
      This function is used to test if a point is in frustums
      */
      bool isInsideFRUSTUMs(pcl::PointXYZI testPoint);
      
    private:
      std::vector<costmap_depth_camera::Observation>* observations_;
      rclcpp::Logger logger_;
  
  };    /// class FrustumUtils
}       /// namespace costmap_depth_camera
#endif  /// COSTMAP_DEPTH_CAMERA_FRUSTUM_UTILS_H_ 
