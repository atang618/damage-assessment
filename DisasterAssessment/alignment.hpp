//
//  alignment.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef alignment_hpp
#define alignment_hpp

#include <stdio.h>
#include "utilities.hpp"

template <typename PointSource, typename PointTarget, typename Scalar = float>
class IterativeClosestPoint_Exposed : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
    pcl::CorrespondencesPtr getCorrespondencesPtr();
};

double sacIA(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
             const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
             const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
             Eigen::Matrix4f &initial_T,
             bool fine);

void sacIAScaled(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
                 const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
                 const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
                 Eigen::Matrix4f &T_initial,
                 bool fine,
                 double &in_out_s,
                 int num_iterations,
                 double iteration_scale_step);

void initialAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
                      const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
                      const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
                      Eigen::Matrix4f &T_initial,
                      double& max_s,
                      int num_iterations,
                      double iteration_scale_step);

#endif /* alignment_hpp */
