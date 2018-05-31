//
//  alignment.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 4/25/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "alignment.hpp"

template<> pcl::CorrespondencesPtr IterativeClosestPoint_Exposed<pcl::PointXYZ, pcl::PointXYZ>::getCorrespondencesPtr()
{
    //        for (uint32_t i = 0; i < this->correspondences_->size(); i++) {
    //            pcl::Correspondence currentCorrespondence = (*this->correspondences_)[i];
    //            std::cout << "Index of the source point: " << currentCorrespondence.index_query << std::endl;
    //            std::cout << "Index of the matching target point: " << currentCorrespondence.index_match << std::endl;
    //            std::cout << "Distance between the corresponding points: " << currentCorrespondence.distance << std::endl;
    //            std::cout << "Weight of the confidence in the correspondence: " << currentCorrespondence.weight << std::endl;
    //        }
    return this->correspondences_;
}

double sacIA(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
             const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
             pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
             const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
             const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
             Eigen::Matrix4f &T_initial,
             bool fine){
    // Initialize Sample Consensus Initial Alignment (SAC-IA)
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> reg;
    reg.setMinSampleDistance (0.05f); //0.05f default
    if (fine) {
        reg.setMaxCorrespondenceDistance (0.05); //0.0001
        reg.setMaximumIterations (30000);
    } else {
        reg.setMaxCorrespondenceDistance (0.05); // 0.01 default
        reg.setMaximumIterations (2000);
    }

    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>::Ptr rej_samp (new pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ>);
    reg.addCorrespondenceRejector(rej_samp);
    reg.setEuclideanFitnessEpsilon(0.00001);
//    reg.setNumberOfSamples(6);
//    std::cout << reg.getCorrespondenceRandomness();
    // Set source and target
    reg.setInputCloud (source);
    reg.setInputTarget (target);
    reg.setSourceFeatures (features_source.makeShared ());
    reg.setTargetFeatures (features_target.makeShared ());

    // Register
    reg.align (*source_align);
    T_initial = reg.getFinalTransformation();
    return reg.getFitnessScore();
}

//template<typename FeatureType>
//double transformationEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
//                                const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
//                                pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
//                                const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
//                                const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
//                                Eigen::Matrix4f &T_initial){
//    
//}

void sacIAScaled(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source,
                 const pcl::PointCloud<pcl::PointXYZ>::Ptr &target,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
                 const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
                 const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
                 Eigen::Matrix4f &T_initial,
                 bool fine,
                 double &in_out_s,
                 int num_iterations,
                 double iteration_scale_step)
{
    double s = in_out_s;
    double min_score = 1.0; Eigen::Matrix4f min_T; double min_s = s;
    pcl::PointCloud<pcl::PointXYZ>::Ptr min_source_align (new pcl::PointCloud<pcl::PointXYZ>());
    for(int i=-(num_iterations/2);i<=(num_iterations/2);i++)
        //int i=0;
    {
        double _s = (s + (double)i*(s*iteration_scale_step));
        if (_s == 0) {
            continue;
        }
        std::cout << "scale synth to " << _s << std::endl;
        Eigen::Matrix4f T_s = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
        T_s.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * _s;
        std::cout << "apply scale"<<std::endl<<T_s<<std::endl;
        
        pcl::PointCloud<pcl::PointXYZ> source_s;
        pcl::transformPointCloud<pcl::PointXYZ>(*source, source_s, T_s);
        
        double score = sacIA(source_s.makeShared(),target, source_align, features_source, features_target, T_initial, fine);
        std::cout << "Initial transform:"<<std::endl<<T_initial.transpose()<<std::endl;
        std::cout << "Fitness score:"<<score<<std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        
        if(score < min_score) {
            min_score = score;
            min_T = T_initial;
            min_s = _s;
            *min_source_align = *source_align;
        }
    }
    T_initial = min_T;
    in_out_s = min_s;
    *source_align = *min_source_align;
}


void initialAlignment(const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
                      const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& source_align,
                      const pcl::PointCloud<pcl::FPFHSignature33> &features_source,
                      const pcl::PointCloud<pcl::FPFHSignature33> &features_target,
                      Eigen::Matrix4f &T_initial,
                      double& max_s,
                      int num_iterations,
                      double iteration_scale_step)
{
    // Perform eigenvalue decomposition to determine scale of point clouds
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(source);
    Eigen::Vector4f v_A_mu = pca.getMean();
    Eigen::Vector3f ev_A = pca.getEigenValues();
    
    pca.setInputCloud(target);
    Eigen::Vector4f v_B_mu = pca.getMean();
    Eigen::Vector3f ev_B = pca.getEigenValues();
    
    // Take average of three eigenvalues directions
    double s = ((sqrt(ev_B[0])/sqrt(ev_A[0]))+(sqrt(ev_B[1])/sqrt(ev_A[1]))+(sqrt(ev_B[2])/sqrt(ev_A[2])))/3;
//    double s = ((sqrt(ev_B[0])/sqrt(ev_A[0])));
    std::cout << "Initial Scale: " << s << std::endl;
    //rough
    sacIAScaled(source, target, source_align, features_source, features_target, T_initial, false, s, num_iterations, iteration_scale_step);
    max_s = s;
    
    //fine
    sacIAScaled(source,target, source_align, features_source, features_target, T_initial, true, max_s ,num_iterations, iteration_scale_step/10.0);
    
    std::cout << "Final Scale: " << max_s << std::endl;
}
