//
//  utilities.hpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/1/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#ifndef utilities_hpp
#define utilities_hpp

#include <stdio.h>
#include <fstream>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/registration.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/features/ppf.h>
#include <pcl/registration/ppf_registration.h>
#include <pcl/registration/ndt.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/eigen.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/features/shot.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/Core>
#include <Eigen/Dense>


#define MAXBUFSIZE  20

void saveEigenMatrix(Eigen::Matrix4f &src, std::string pathname);
void loadEigenMatrix(Eigen::Matrix4f &src, std::string pathname);

#endif /* utilities_hpp */
