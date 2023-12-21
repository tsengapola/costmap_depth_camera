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

#include <nav2_costmap_2d/frustum_utils.hpp>
#define DEBUG 0

namespace nav2_costmap_2d
{

  FrustumUtils::FrustumUtils(std::vector<nav2_costmap_2d::ObservationDepth>* observations)
  : observations_(observations)
  , logger_(rclcpp::get_logger("frustum_utils_logger"))
  {

  }

  FrustumUtils::~FrustumUtils()
  {

  }

  bool FrustumUtils::isAttachFRUSTUMs(pcl::PointXYZI testPoint)
  {
    for (std::vector<nav2_costmap_2d::ObservationDepth>::iterator it = (*observations_).begin(); it != (*observations_).end(); ++it)
    {
      nav2_costmap_2d::ObservationDepth& obs = *it;
      for(auto it_plane=obs.frustum_plane_equation_.begin();it_plane!=obs.frustum_plane_equation_.end();it_plane++)
      {
        float a = (*it_plane)[0];
        float b = (*it_plane)[1];
        float c = (*it_plane)[2];
        float d = (*it_plane)[3];
        float dis = fabs(a*testPoint.x+b*testPoint.y+c*testPoint.z+d);
        dis = dis/sqrt(a*a+b*b+c*c);
        float dis2rej = 0.12;

        if(dis <= dis2rej && hypot(testPoint.x-obs.origin_.x, testPoint.y-obs.origin_.y)<obs.max_detect_distance_+0.5)
        {
        //find one frustum such that no attachment and inside frumstum
          for (std::vector<nav2_costmap_2d::ObservationDepth>::iterator it_inner = (*observations_).begin(); it_inner != (*observations_).end(); ++it_inner)
          {
            if(it==it_inner)
            {
            //ROS_WARN("Same frustum.");
              continue;
            }
            else if(isInsideFRUSTUMwoAttach(*it_inner, testPoint))
            {
              return false;
            }
          }
          return true;
        }
      } 
    }
    return false;
  }
  
  bool FrustumUtils::isInsideFRUSTUMwoAttach(nav2_costmap_2d::ObservationDepth& observation, pcl::PointXYZI testPoint)
  {
    for(auto it=observation.frustum_plane_equation_.begin();it!=observation.frustum_plane_equation_.end();it++)
    {
      float a = (*it)[0];
      float b = (*it)[1];
      float c = (*it)[2];
      float d = (*it)[3];
      float dis = fabs(a*testPoint.x+b*testPoint.y+c*testPoint.z+d);
      dis = dis/sqrt(a*a+b*b+c*c);
      float dis2rej = 0.12;
      if(dis<=dis2rej && hypot(testPoint.x-observation.origin_.x, testPoint.y-observation.origin_.y)<observation.max_detect_distance_+0.5)
      {
        return false;
      }
    } 

    pcl::PointXYZ testPoint_XYZ;
    testPoint_XYZ.x = testPoint.x;
    testPoint_XYZ.y = testPoint.y;
    testPoint_XYZ.z = testPoint.z;

    pcl::PointXYZ vec_form_pc2corner;
    double test;
    for(int i=0;i<6;i++)
    {
      test = 0.0;
      if(i<3)
      {
        vec_form_pc2corner = observation.getVec(observation.BRNear_,testPoint_XYZ);
        //ROS_DEBUG("%d, %.2f, %.2f, %.2f",i, vec_form_pc2corner.point.x,vec_form_pc2corner.point.y,vec_form_pc2corner.point.z);
        test = vec_form_pc2corner.x*observation.frustum_normal_->points[i].x + vec_form_pc2corner.y*observation.frustum_normal_->points[i].y +vec_form_pc2corner.z*observation.frustum_normal_->points[i].z;
        if(test<0)
        {
          //ROS_DEBUG("Reject by %d plane.", i);
          return false;
        }
      }
      else
      {
        vec_form_pc2corner = observation.getVec(observation.TLFar_,testPoint_XYZ);
        //ROS_DEBUG("%d: %.2f, %.2f, %.2f",i, vec_form_pc2corner.point.x,vec_form_pc2corner.point.y,vec_form_pc2corner.point.z);
        test = vec_form_pc2corner.x*observation.frustum_normal_->points[i].x + vec_form_pc2corner.y*observation.frustum_normal_->points[i].y +vec_form_pc2corner.z*observation.frustum_normal_->points[i].z;
        if(test<0)
        {
          //ROS_DEBUG("Reject by %d plane.", i);
          return false;
        }
      }
    }
    return true;
  }

  bool FrustumUtils::isInsideFRUSTUMs(pcl::PointXYZI testPoint)
  {
    pcl::PointXYZ testPoint_XYZ;
    testPoint_XYZ.x = testPoint.x;
    testPoint_XYZ.y = testPoint.y;
    testPoint_XYZ.z = testPoint.z;

    for (std::vector<nav2_costmap_2d::ObservationDepth>::iterator it = (*observations_).begin(); it != (*observations_).end(); ++it)
    {  
    bool one_frustum_test = true; //if we test the point in one of the frustum, then we can exit
    nav2_costmap_2d::ObservationDepth& obs = *it;
    pcl::PointXYZ vec_form_pc2corner;
    double test;
    for(int i=0;i<6;i++)
    {
      test = 0.0;
      if(i<3)
      {
        vec_form_pc2corner = obs.getVec(obs.BRNear_,testPoint_XYZ);
        
        #if(DEBUG)
        RCLCPP_INFO(logger_,"%d, %.2f, %.2f, %.2f",i, vec_form_pc2corner.x,vec_form_pc2corner.y,vec_form_pc2corner.z);
        #endif
        
        test = vec_form_pc2corner.x*obs.frustum_normal_->points[i].x + vec_form_pc2corner.y*obs.frustum_normal_->points[i].y +vec_form_pc2corner.z*obs.frustum_normal_->points[i].z;
        
        if(test<0)
        {
          #if(DEBUG)
          RCLCPP_INFO(logger_,"Reject by %d plane.", i);
          #endif
          one_frustum_test = false;
          break;
        }
      }
      else
      {
        vec_form_pc2corner = obs.getVec(obs.TLFar_,testPoint_XYZ);
        
        #if(DEBUG)
        RCLCPP_INFO(logger_,"%d: %.2f, %.2f, %.2f",i, vec_form_pc2corner.x,vec_form_pc2corner.y,vec_form_pc2corner.z);
        #endif

        test = vec_form_pc2corner.x*obs.frustum_normal_->points[i].x + vec_form_pc2corner.y*obs.frustum_normal_->points[i].y +vec_form_pc2corner.z*obs.frustum_normal_->points[i].z;
        if(test<0)
        {
          #if(DEBUG)
          RCLCPP_INFO(logger_,"Reject by %d plane.", i);
          #endif
          one_frustum_test = false;
          break;
        }
      }
    }
    if(one_frustum_test)
    {
      return true;
    }
  }
  return false;
  }

} /// namespace nav2_costmap_2d