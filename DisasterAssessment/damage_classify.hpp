//
//  damage_classify.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef damage_classify_hpp
#define damage_classify_hpp

#include <stdio.h>
#include "utilities.hpp"

std::vector<pcl::PointIndices> findClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double cluster_tol, double min_size, double max_size);
void processClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<pcl::PointIndices>& indices, std::string directory, std::string tag,
                     double cloud_variance, double thresh1, double thresh2);

#endif /* damage_classify_hpp */
