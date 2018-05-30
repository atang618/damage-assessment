//
//  plane_segment.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "plane_segment.hpp"

void removeIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out,
                   const pcl::PointIndices::Ptr &indices) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_in);
    extract.setIndices (indices);
    extract.setNegative (true);
    extract.filter(*cloud_out);
}
                   
void findPlaneIndicesSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, double distance) {
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
    std::vector<int> indices;
    Eigen::VectorXf coefficients;
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_plane);
    ransac.setDistanceThreshold (distance);
    ransac.computeModel();
    ransac.getInliers(indices);
    ransac.getModelCoefficients(coefficients);
    
    inliers->indices = indices;

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << coefficients[0] << " "
    << coefficients[1] << " "
    << coefficients[2] << " "
    << coefficients[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    << cloud->points[inliers->indices[i]].y << " "
    << cloud->points[inliers->indices[i]].z << std::endl;
}

void findPlaneIndicesSeg(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, double angle) {
    // Compute normals
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
    
    // Initialize Region Growing
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (5000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (angle / 180.0 * M_PI);
    reg.setCurvatureThreshold (3.0);
    
    // Extract region indices
    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    // Find the largest cluster and save as the inliers
    int maxClusterInd = 0;
    size_t maxClusterSize = clusters[0].indices.size();
    for (int i = 1; i < clusters.size(); i++) {
        if (clusters[i].indices.size() > maxClusterSize) {
            maxClusterSize = clusters[i].indices.size();
            maxClusterInd = i;
        }
    }
    
    *inliers = clusters[0];
    
}

void removePlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, double thresh) {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    findPlaneIndicesSAC(cloud_in, inliers, thresh);
//    findPlaneIndicesSeg(cloud_in, inliers, thresh);
    removeIndices(cloud_in, cloud_out, inliers);
}
