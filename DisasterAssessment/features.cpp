//
//  features.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/1/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "features.hpp"

void computeFPFHFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints, pcl::PointCloud<pcl::FPFHSignature33> &features) {
    // Initialize estimators for surface normals and FPFH features
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (0.05);
    pcl::PointCloud<pcl::Normal> normals;

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
    fpfh_est.setSearchMethod (tree);
    fpfh_est.setRadiusSearch (0.05);

    // Estimate the FPFH features for the cloud
    norm_est.setInputCloud (keypoints);
    norm_est.compute (normals);
    fpfh_est.setInputCloud (keypoints);
    fpfh_est.setInputNormals (normals.makeShared ());
    fpfh_est.compute (features);
}

void computeSHOTColorFeatures(const pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints, pcl::PointCloud<pcl::SHOT1344>::Ptr &features) {
    // Compute surface normals
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setSearchMethod (tree);
    norm_est.setRadiusSearch (0.05);
    pcl::PointCloud<pcl::Normal> normals;
    norm_est.setInputCloud (keypoints);
    norm_est.compute (normals);
    
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr rgbTree;
    pcl::SHOTColorEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT1344> shot1344 (true, true);
    shot1344.setInputNormals (normals.makeShared());
    
    
}
