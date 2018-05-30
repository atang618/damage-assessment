//
//  damage_classify.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "damage_classify.hpp"

std::vector<pcl::PointIndices> findClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double cluster_tol, double min_size, double max_size) {
    // Group close points together into one region
    pcl::search::KdTree<pcl::PointXYZ>::Ptr ectree (new pcl::search::KdTree<pcl::PointXYZ>);
    ectree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tol);
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size); 
    ec.setSearchMethod (ectree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    
    return cluster_indices;
}

void processClusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const std::vector<pcl::PointIndices>& indices, std::string directory, std::string tag,
                     double cloud_variance, double thresh1, double thresh2) {
    // Return number of regions, center location, and eigenvalues
    pcl::PointCloud<pcl::PointXYZ>::Ptr total_clusters (new pcl::PointCloud<pcl::PointXYZ>);
    std::ofstream clusterInfo;
    clusterInfo.open(directory + "clusterInformation_" + tag + ".txt");
    clusterInfo << "Number of regions: " << indices.size() << "\n";
    for (int i = 0; i < indices.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndices::Ptr cluster_ptr (new pcl::PointIndices ());
        *cluster_ptr = indices[i];
        pcl::ExtractIndices<pcl::PointXYZ> ex;
        ex.setInputCloud(cloud);
        ex.setIndices (cluster_ptr);
        ex.filter(*cluster);
        // Save each cluster and add to the total clusters
        pcl::io::savePLYFileBinary(directory + "cluster" + std::to_string(i) + "_" + tag + ".ply", *cluster);
        *total_clusters += *cluster;
        // Get the centroid
        pcl::PointXYZ centroid;
        pcl::computeCentroid(*cluster, centroid);
        // Perform eigenvalue decomposition to determine "size" of point clouds
        pcl::PCA<pcl::PointXYZ> pca_cluster;
        pca_cluster.setInputCloud(cluster);
        Eigen::Vector4f mu = pca_cluster.getMean();
        Eigen::Vector3f ev = pca_cluster.getEigenValues();
        double variance = (sqrt(ev[0])+sqrt(ev[1])+sqrt(ev[2]))/3;
        double ratio = variance/cloud_variance;
        clusterInfo << "Cluster " << i << ": center: " << centroid << "; Average variance: "<< variance <<
        "; Ratio: " << ratio << "; Number of points: " << indices[i].indices.size() << "\n";
        if (ratio > thresh1) { // 0.05
            clusterInfo << "Damage: fully demolished\n";
        } else if (ratio > thresh2) { // 0.008
            clusterInfo << "Damage: partially demolished\n";
        } else {
            clusterInfo << "Damage: minor damage\n";
        }
    }
    clusterInfo.close();
    // Save total clusters
    pcl::io::savePLYFileBinary(directory + "total_clusters_" + tag + ".ply", *total_clusters);

}
