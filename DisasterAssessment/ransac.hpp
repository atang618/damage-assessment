//
//  ransac.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/22/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef ransac_hpp
#define ransac_hpp

#include <stdio.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
//#include <pcl/console/parse.h>
//#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/sample_consensus/sac_model_registration_2d.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/eigen.h>
#include <Eigen/Core>


using namespace std;

int RANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                   const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                   Eigen::Matrix4f& Tresult);

void ScaleRANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         Eigen::Matrix4f& Tresult,
                         double& max_s,
                         int num_iterations,
                         double iteration_scale_step);

void ScaleRANSACRegister(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudA,
                         const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudB,
                         Eigen::Matrix4f& Tresult,
                         double& max_s,
                         int num_iterations,
                         double iteration_scale_step);

#endif /* ransac_hpp */
