//
//  main.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/22/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include <iostream>

#include "ransac.hpp"
#include "plane_segment.hpp"
#include "alignment.hpp"
#include "utilities.hpp"
#include "keypoints.hpp"
#include "features.hpp"
#include "damage_classify.hpp"

#define PLANE_T 0.04
#define PLANE_S 0.07
#define NUM_ITER 6
#define SCALE_STEP 0.1
#define OUT_THRESH 0.005
#define CLUST_TOL 0.02 // 0.02 default

using namespace pcl;


int main(int argc, const char * argv[]) {
    bool loadKeypoints = false;
    bool loadTransformation = false;
    string keypointsDirectory;
    string transformationFilename;
    string scaleFilename;
    string beforeDirectory = "../../../results/disaster_before/dense/";
    string afterDirectory = "../../../results/disaster_after/dense/";
    string resultsDirectory = "../../../results/alignment/";
    // Check if alignment has already been computed
    if (argc > 1) {
        for (int i = 1; i < argc; i++) {
            string arg = argv[i];
            if (arg == "-k") {
                keypointsDirectory = argv[++i];
                loadKeypoints = true;
            } else if (arg == "-t") {
                transformationFilename = argv[++i];
                loadTransformation = true;
            } else if (arg == "-s") {
                scaleFilename = argv[++i];
            } else {
                std::cerr << "Unknown argument" << std::endl;
            }
        }
    }
    
    PointCloud<PointXYZ>::Ptr before (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr after (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr before_noplane (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr after_noplane (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr source_keypoints (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr target_keypoints (new PointCloud<PointXYZ>);
    if (loadKeypoints) {
        // Load no plane files
        if (io::loadPLYFile(keypointsDirectory + "after_noplane.ply", *after_noplane) == -1) {
            PCL_ERROR ("Couldn't read file after_noplane.ply \n");
            return (-1);
        }
        if (io::loadPLYFile(keypointsDirectory + "before_noplane.ply", *before_noplane) == -1) {
            PCL_ERROR ("Couldn't read file before_noplane.ply \n");
            return (-1);
        }
        // Load keypoints
        if (io::loadPLYFile(keypointsDirectory + "after_keypoints.ply", *source_keypoints) == -1) {
            PCL_ERROR ("Couldn't read file after_keypoints.ply \n");
            return (-1);
        }
        if (io::loadPLYFile(keypointsDirectory + "before_keypoints.ply", *target_keypoints) == -1) {
            PCL_ERROR ("Couldn't read file before_keypoints.ply \n");
            return (-1);
        }
    } else {
        // Load raw data
        if (io::loadPLYFile(beforeDirectory + "scene_dense_ASCII.ply", *before) == -1) {
            PCL_ERROR ("Couldn't read file before.ply \n");
            return (-1);
        }
        if (io::loadPLYFile(afterDirectory + "scene_dense_ASCII.ply", *after) == -1) {
            PCL_ERROR ("Couldn't read file after.ply \n");
            return (-1);
        }
        
        // Filter out the plane from each save the resulting ply file
        removePlane(before, before_noplane, PLANE_T); //0.01
        io::savePLYFileBinary(resultsDirectory + "before_noplane.ply",
                              *before_noplane);
        removePlane(after, after_noplane, PLANE_S); //0.01
        io::savePLYFileBinary(resultsDirectory + "after_noplane.ply",
                              *after_noplane);
        
//        copyPointCloud(*after, *after_noplane);
//        copyPointCloud(*before, *before_noplane);
        
        // Compute keypoints using ISS
        computeISSKeypoints(after_noplane, source_keypoints);
        computeISSKeypoints(before_noplane, target_keypoints);
        
        // Save keypoints
        io::savePLYFileBinary(resultsDirectory + "after_keypoints.ply", *source_keypoints);
        io::savePLYFileBinary(resultsDirectory + "before_keypoints.ply", *target_keypoints);
    }
    
    // Compute variance of target noplane point cloud
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(before_noplane);
    Eigen::Vector4f muB = pca.getMean();
    Eigen::Vector3f evB = pca.getEigenValues();
    double target_variance = (sqrt(evB[0])+sqrt(evB[1])+sqrt(evB[2]))/3;
    std::ofstream targetInfo;
    targetInfo.open(resultsDirectory + "targetInformation.txt");
    targetInfo << "Eigenvalues: " << evB << "\n";
    targetInfo << "Average variance: " << target_variance << "\n";
    targetInfo.close();
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
    cloud_source_ptr = source_keypoints->makeShared ();
    cloud_target_ptr = target_keypoints->makeShared ();
    Eigen::Matrix4f T_s, initial_T;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_align_initial (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints_transformed_initial (new pcl::PointCloud<PointXYZ>);
    if (loadTransformation) {
        loadEigenMatrix(T_s, scaleFilename);
        loadEigenMatrix(initial_T, transformationFilename);
        if (io::loadPLYFile(keypointsDirectory + "source_keypoints_transformed_initial.ply", *source_keypoints_transformed_initial) == -1) {
            PCL_ERROR ("Couldn't read file source_keypoints_transformed_initial.ply \n");
            return (-1);
        }

    }
    else {
        // Compute FPFH features
        pcl::PointCloud<pcl::FPFHSignature33> features_source, features_target;
        computeFPFHFeatures(cloud_source_ptr, features_source);
        computeFPFHFeatures(cloud_target_ptr, features_target);

        
        // Perform Sample Consensus Initial Alignment (SAC-IA) at various scales
        double max_s;
        initialAlignment(cloud_source_ptr, cloud_target_ptr, source_keypoints_align_initial, features_source, features_target, initial_T, max_s, NUM_ITER, SCALE_STEP);
        
        //    Eigen::Matrix4f refined_T;
        //    double max_s;
        //    ScaleRANSACRegister(source_keypoints, target_keypoints, refined_T, max_s, NUM_ITER, SCALE_STEP);
        
        T_s = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
        T_s.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * max_s;
        
        // Save initial transformation, scale and aligned keypoints for future reference
        saveEigenMatrix(initial_T, resultsDirectory + "initialTransformation.txt");
        saveEigenMatrix(T_s, resultsDirectory + "scaleTransformation.txt");
        io::savePLYFileBinary(resultsDirectory + "source_keypoints_align_initial.ply", *source_keypoints_align_initial);
        
        // Apply scale
        pcl::PointCloud<pcl::PointXYZ>::Ptr initial_scaled (new PointCloud<PointXYZ>);
        pcl::transformPointCloud<pcl::PointXYZ>(*cloud_source_ptr, *initial_scaled, T_s);
        // Apply transformation
        transformPointCloud(*initial_scaled, *source_keypoints_transformed_initial, initial_T);
        io::savePLYFileBinary(resultsDirectory + "source_keypoints_transformed_initial.ply", *source_keypoints_transformed_initial);
        
    }

    // Refine with Iterative Closest Point
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    icp.setMaximumIterations(400000);
    icp.setMaxCorrespondenceDistance(OUT_THRESH*10);
    icp.setRANSACOutlierRejectionThreshold(OUT_THRESH*2);
    icp.setEuclideanFitnessEpsilon(0.000001);
    icp.setInputCloud (source_keypoints_transformed_initial);
    icp.setInputTarget (cloud_target_ptr);
    PointCloud<PointXYZ>::Ptr source_keypoints_align_final (new PointCloud<PointXYZ>);
    icp.align (*source_keypoints_align_final);
    cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
    Eigen::Matrix4f refined_T = icp.getFinalTransformation() * initial_T;
    cout << refined_T << endl;
    io::savePLYFileBinary(resultsDirectory + "source_keypoints_align_final.ply", *source_keypoints_align_final);
    io::savePLYFileBinary(resultsDirectory + "target_keypoints.ply", *cloud_target_ptr);
    
    
//    // Perform ICP with entire noplane clouds
//    PointCloud<PointXYZ>::Ptr total_noplane_align (new PointCloud<PointXYZ>);
//    PointCloud<PointXYZ>::Ptr total_noplane_scaled (new PointCloud<PointXYZ>);
//    transformPointCloud<pcl::PointXYZ>(*after_noplane, *total_noplane_scaled, T_s);
//    transformPointCloud(*total_noplane_scaled, *total_noplane_align, refined_T);
//    io::savePLYFileBinary("../../../results/alignment/after_noplane_initial_align.ply", *total_noplane_align);
//    IterativeClosestPoint_Exposed<PointXYZ, PointXYZ> icp_total;
//    icp_total.setInputCloud (total_noplane_align);
//    icp_total.setInputTarget (before_noplane);
//    PointCloud<PointXYZ>::Ptr total_noplane_align_final (new PointCloud<PointXYZ>);
//    icp_total.align(*total_noplane_align_final);
//    cout << "has converged:" << icp_total.hasConverged() << " score: " <<
//    icp_total.getFitnessScore() << std::endl;
//    Eigen::Matrix4f final_T = icp_total.getFinalTransformation() * refined_T;
//    cout << final_T << endl;
    
    
//    // Extract outliers from rigid transformation
//    pcl::CorrespondencesPtr correspondences = icp_total.getCorrespondencesPtr();
//    pcl::PointIndices::Ptr source_matches (new pcl::PointIndices);
//    std::vector<int> match_indices;
//    for (int i = 0; i < correspondences->size(); i++) {
//        pcl::Correspondence currentCorrespondence = (*correspondences)[i];
//        if (currentCorrespondence.distance < OUT_THRESH) {
//            match_indices.push_back(currentCorrespondence.index_query);
//        }
//    }
//    source_matches->indices = match_indices;
//    pcl::ExtractIndices<pcl::PointXYZ> extract (true);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr source_align_filter (new pcl::PointCloud<PointXYZ>);
//    extract.setInputCloud(total_noplane_align_final);
//    extract.setIndices (source_matches);
//    extract.setNegative (true);
//    extract.filter (*source_align_filter);
//    pcl::IndicesConstPtr source_outliers = extract.getRemovedIndices();
    
    // Align total point clouds and find differences
    pcl::PointCloud<PointXYZ>::Ptr total_noplane_scale (new PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr total_noplane_align (new PointCloud<PointXYZ>);
    transformPointCloud(*after_noplane, *total_noplane_scale, T_s);
    transformPointCloud(*total_noplane_scale, *total_noplane_align, refined_T);
    io::savePLYFileBinary(resultsDirectory + "total_noplane_align.ply", *total_noplane_align);
    
    // Find differences with respect to before disaster
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::SegmentDifferences<pcl::PointXYZ> seg;
    seg.setSearchMethod(tree);
    seg.setDistanceThreshold(OUT_THRESH);
    seg.setInputCloud(before_noplane);
    seg.setTargetCloud(total_noplane_align);
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_differences_before (new pcl::PointCloud<PointXYZ>);
    seg.segment(*seg_differences_before);
    
    // Find differences with respect to after disaster
    seg.setInputCloud(total_noplane_align);
    seg.setTargetCloud(before_noplane);
    pcl::PointCloud<pcl::PointXYZ>::Ptr seg_differences_after (new pcl::PointCloud<PointXYZ>);
    seg.segment(*seg_differences_after);
    
    // Save segmented differences
    io::savePLYFileBinary(resultsDirectory + "seg_differences_before.ply", *seg_differences_before);
    io::savePLYFileBinary(resultsDirectory + "seg_differences_after.ply", *seg_differences_after);
    
//    // Paint these as red
//    pcl::PointCloud<PointXYZRGB>::Ptr total_noplane_align_color (new PointCloud<PointXYZRGB>);
//    copyPointCloud(*total_noplane_align_final, *total_noplane_align_color);
//    for (int i = 0; i < source_outliers->size(); i++) {
//        total_noplane_align_color->points[(*source_outliers)[i]].b = 255;
//    }
//    io::savePLYFileBinary("../../../results/alignment/aligned_noplane_after_color.ply", *total_noplane_align_color);
    
    // Find valid damage clusters
    std::vector<pcl::PointIndices> cluster_indices_before;
    std::vector<pcl::PointIndices> cluster_indices_after;
    cluster_indices_before = findClusters(seg_differences_before, CLUST_TOL, 300, 1000000);
    cluster_indices_after = findClusters(seg_differences_after, CLUST_TOL, 300, 1000000);

    // Save each cluster and its information
    processClusters(seg_differences_before, cluster_indices_before, resultsDirectory, "before",
                    target_variance, 0.05, 0.008);
    processClusters(seg_differences_after, cluster_indices_after, resultsDirectory, "after",
                    target_variance, 0.05, 0.008);
    
    // Align total point cloud with color for display purposes
    pcl::PointCloud<PointXYZRGB>::Ptr after_color (new PointCloud<PointXYZRGB>);
    if (io::loadPLYFile(afterDirectory+"scene_dense_ASCII.ply", *after_color) == -1) {
        PCL_ERROR ("Couldn't read file after.ply \n");
        return (-1);
    }
    pcl::PointCloud<PointXYZRGB>::Ptr after_color_scaled (new PointCloud<PointXYZRGB>);
    pcl::PointCloud<PointXYZRGB>::Ptr after_color_aligned (new PointCloud<PointXYZRGB>);
    transformPointCloud(*after_color, *after_color_scaled, T_s);
    transformPointCloud(*after_color_scaled, *after_color_aligned, refined_T);
    io::savePLYFileBinary(resultsDirectory + "after_color_aligned.ply", *after_color_aligned);
    
    return 0;
}
