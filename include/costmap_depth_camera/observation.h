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

#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
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
  Observation() :
    cloud_(new pcl::PointCloud<pcl::PointXYZI>()), frustum_(new pcl::PointCloud<pcl::PointXYZ>()),
    frustum_normal_(new pcl::PointCloud<pcl::PointXYZ>()), frustum_plane_equation_(), 
    min_detect_distance_(0.01), max_detect_distance_(2.5), FOV_W_(1.5), FOV_V_(1.0)
  {
  }

  /**
   * @brief  Copy constructor
   * @param obs The observation to copy
   */
  Observation(const Observation& obs) :
      origin_(obs.origin_), cloud_(new pcl::PointCloud<pcl::PointXYZI>(*(obs.cloud_))),
      frustum_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.frustum_))), frustum_normal_(new pcl::PointCloud<pcl::PointXYZ>(*(obs.frustum_normal_))),
      frustum_plane_equation_(obs.frustum_plane_equation_),
      min_detect_distance_(obs.min_detect_distance_), max_detect_distance_(obs.max_detect_distance_),
      FOV_V_(obs.FOV_V_), FOV_W_(obs.FOV_W_), BRNear_(obs.BRNear_), TLFar_(obs.TLFar_)
  {
  }

  virtual ~Observation()
  {
    delete cloud_;
    delete frustum_;
    delete frustum_normal_;
  }

  geometry_msgs::Point origin_;
  pcl::PointCloud<pcl::PointXYZI>* cloud_;
  pcl::PointCloud<pcl::PointXYZ>* frustum_;
  pcl::PointCloud<pcl::PointXYZ>* frustum_normal_;
  std::vector<Eigen::Vector4f> frustum_plane_equation_;

  /*
  These parameter is essential for depth camera
  */
  double min_detect_distance_, max_detect_distance_, FOV_W_, FOV_V_;
  
  /*
  These points are for frustum check
  */
  pcl::PointXYZ BRNear_,TLFar_;

  pcl::PointXYZ getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2){
    pcl::PointXYZ vec;
    vec.x = vec2.x - vec1.x;
    vec.y = vec2.y - vec1.y;
    vec.z = vec2.z - vec1.z;
    return vec;
  }

  pcl::PointXYZ getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2){
    pcl::PointXYZ crsp;
    crsp.x = vec1.y * vec2.z - vec1.z * vec2.y;
    crsp.y = (vec1.x * vec2.z - vec1.z * vec2.x)*-1.0;
    crsp.z = vec1.x * vec2.y - vec1.y * vec2.x;
    //cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]; 
    //cross_P[1] = vect_A[0] * vect_B[2] - vect_A[2] * vect_B[0]; 
    //cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]; 

    return crsp;
  }

  // Function to find equation of plane. 
  void getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3) 
  { 
    plane_equation = Eigen::Vector4f::Zero();
    float a1 = p2.x - p1.x; 
    float b1 = p2.y - p1.y; 
    float c1 = p2.z - p1.z; 
    float a2 = p3.x - p1.x; 
    float b2 = p3.y - p1.y; 
    float c2 = p3.z - p1.z; 
    plane_equation[0] = b1 * c2 - b2 * c1; 
    plane_equation[1] = a2 * c1 - a1 * c2; 
    plane_equation[2] = a1 * b2 - b1 * a2; 
    plane_equation[3] = (-plane_equation[0] * p1.x - plane_equation[1] * p1.y - plane_equation[2] * p1.z); 
  } 

  void findFrustumVertex()
  {
    

    frustum_->clear();

    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));
    
    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), min_detect_distance_*tan(FOV_V_/2.0)));

    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));

    frustum_->push_back(pcl::PointXYZ(min_detect_distance_, -min_detect_distance_*tan(FOV_W_/2.0), -min_detect_distance_*tan(FOV_V_/2.0)));

    //-------------------------------------------

    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));
    
    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), max_detect_distance_*tan(FOV_V_/2.0)));

    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));

    frustum_->push_back(pcl::PointXYZ(max_detect_distance_, -max_detect_distance_*tan(FOV_W_/2.0), -max_detect_distance_*tan(FOV_V_/2.0)));

  }

  void findFrustumNormal()
  {

    BRNear_ = frustum_->points[3];
    TLFar_ = frustum_->points[4];
    
    frustum_normal_->clear();
    
    pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;

    TLNear = frustum_->points[0];

    TRNear = frustum_->points[1];

    BLNear = frustum_->points[2];

    BRNear = frustum_->points[3];


    pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;
    
    TLFar = frustum_->points[4];

    TRFar = frustum_->points[5];

    BLFar = frustum_->points[6];

    BRFar = frustum_->points[7];

    pcl::PointXYZ TLN2TRN, TRN2BRN; //note it is vector, we manipulate point as vector
    pcl::PointXYZ TRN2TRF, TRF2BRF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BRN2BRF, BRF2BLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BLN2BLF, BLF2TLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ BRF2TRF, TRF2TLF; //note it is vector, we manipulate point as vector
    pcl::PointXYZ TLF2TRF, TRF2TRN; //note it is vector, we manipulate point as vector

    pcl::PointXYZ pt, pb, pl, pr, pn, pf;
    
    ////
    TLN2TRN = getVec(TLNear,TRNear);
    TRN2BRN = getVec(TRNear,BRNear);
    pn = getCrossProduct(TLN2TRN, TRN2BRN);
    frustum_normal_->push_back(pn);
    ////
    TRN2TRF = getVec(TRNear,TRFar);
    TRF2BRF = getVec(TRFar,BRFar);
    pr = getCrossProduct(TRN2TRF, TRF2BRF);
    frustum_normal_->push_back(pr);
    ////
    BRN2BRF = getVec(BRNear,BRFar);
    BRF2BLF = getVec(BRFar,BLFar);
    pb = getCrossProduct(BRN2BRF,BRF2BLF);
    frustum_normal_->push_back(pb);
    ////

    BLN2BLF = getVec(BLNear,BLFar);
    BLF2TLF = getVec(BLFar,TLFar);
    pl = getCrossProduct(BLN2BLF, BLF2TLF);
    frustum_normal_->push_back(pl);
    ////
    BRF2TRF = getVec(BRFar,TRFar);
    TRF2TLF = getVec(TRFar,TLFar);
    pf = getCrossProduct(BRF2TRF, TRF2TLF);
    frustum_normal_->push_back(pf);
    ////
    TLF2TRF = getVec(TLFar,TRFar);
    TRF2TRN = getVec(TRFar,TRNear);
    pt = getCrossProduct(TLF2TRF, TRF2TRN);
    frustum_normal_->push_back(pt);
    /*
    ROS_ERROR("TLNear: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",TLNear.point.x, TLNear.point.y, TLNear.point.z);
    ROS_ERROR("TRNear: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",TRNear.point.x, TRNear.point.y, TRNear.point.z);
    ROS_ERROR("BLNear: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",BLNear.point.x, BLNear.point.y, BLNear.point.z);
    ROS_ERROR("BRNear: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",BRNear.point.x, BRNear.point.y, BRNear.point.z);
    ROS_ERROR("TLFar: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",TLFar.point.x, TLFar.point.y, TLFar.point.z);
    ROS_ERROR("TRFar: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",TRFar.point.x, TRFar.point.y, TRFar.point.z);
    ROS_ERROR("BRFar: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",BRFar.point.x, BRFar.point.y, BRFar.point.z);
    ROS_ERROR("BLFar: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",BLFar.point.x, BLFar.point.y, BLFar.point.z);
    */

    /*
    ROS_WARN("pn: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pn.point.x, pn.point.y, pn.point.z);
    ROS_WARN("pr: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pr.point.x, pr.point.y, pr.point.z);
    ROS_WARN("pb: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pb.point.x, pb.point.y, pb.point.z);
    ROS_WARN("pl: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pl.point.x, pl.point.y, pl.point.z);
    ROS_WARN("pf: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pf.point.x, pf.point.y, pf.point.z);
    ROS_WARN("pt: %.2f >>>>>>>>>,%.2f >>>>>>>>>>>, %.2f",pt.point.x, pt.point.y, pt.point.z);
    */
    
  }

  void findFrustumPlane()
  {
    //frustum_plane_equation_.clear();

    pcl::PointXYZ TLNear,TRNear, BLNear, BRNear;

    TLNear = frustum_->points[0];

    TRNear = frustum_->points[1];

    BLNear = frustum_->points[2];

    BRNear = frustum_->points[3];


    pcl::PointXYZ TLFar,TRFar, BLFar, BRFar;
    
    TLFar = frustum_->points[4];

    TRFar = frustum_->points[5];

    BLFar = frustum_->points[6];

    BRFar = frustum_->points[7];
    
    //we manipulate orientation x,y,z,w as a,b,c,d in plane equation
    Eigen::Vector4f plane1;
    Eigen::Vector4f plane2;
    Eigen::Vector4f plane3;
    Eigen::Vector4f plane4;
    Eigen::Vector4f plane5;
    Eigen::Vector4f plane6;
     
    getPlaneN(plane1,TLNear,TLFar,BLNear);
    getPlaneN(plane2,BLNear,BRNear,BLFar);
    getPlaneN(plane3,TRNear,BRNear,BRFar);
    getPlaneN(plane4,TLNear,TRNear,TLFar);
    getPlaneN(plane5,TLNear,BLNear,BRNear);
    getPlaneN(plane6,TLFar,TRFar,BRFar);
    frustum_plane_equation_.push_back(plane1);
    frustum_plane_equation_.push_back(plane2);
    frustum_plane_equation_.push_back(plane3);
    frustum_plane_equation_.push_back(plane4);
    frustum_plane_equation_.push_back(plane5);
    frustum_plane_equation_.push_back(plane6);


  }

};

}  // namespace costmap_depth_camera
#endif  // COSTMAP_DEPTH_CAMERA_OBSERVATION_H_
