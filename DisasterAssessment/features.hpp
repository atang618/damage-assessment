//
//  features.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/1/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef features_hpp
#define features_hpp

#include <stdio.h>
#include "utilities.hpp"

void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints, pcl::PointCloud<pcl::FPFHSignature33> &features);

#endif /* features_hpp */
