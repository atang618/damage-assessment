//
//  plane_segment.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef plane_segment_hpp
#define plane_segment_hpp

#include <stdio.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include "utilities.hpp"

void removeIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, const pcl::PointIndices &indices);

void findPlaneIndicesSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, double distance);

void findPlaneIndicesSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, double angle);

void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, double thresh);

#endif /* plane_segment_hpp */
