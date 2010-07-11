#include <pluginlib/class_list_macros.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/normal_estimation.h>
#include <cloud_algos/planar_estimation.h>
#include <cloud_algos/rotational_estimation.h>
#include <cloud_algos/region_growing.h>
#include <cloud_algos/cylinder_fit_algo.h>
#include <cloud_algos/box_fit_algo.h>
#include <cloud_algos/box_fit2_algo.h>
#include <cloud_algos/depth_image_triangulation.h>
#include <cloud_algos/mls_fit.h>
#include <cloud_algos/noise_removal.h>
#include <cloud_algos/svm_classification.h>
#include <cloud_algos/global_rsd.h>

using namespace cloud_algos;

//register NormalEstimation as a PCD_Algo class
//PLUGINLIB_REGISTER_CLASS(NormalEstimation, NormalEstimation, PCD_Algo)
//PLUGINLIB_REGISTER_CLASS(PlanarEstimation, PlanarEstimation, PCD_Algo)


PLUGINLIB_DECLARE_CLASS(cloud_algos, MovingLeastSquares, cloud_algos::MovingLeastSquares, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, RotationalEstimation, cloud_algos::RotationalEstimation, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, RegionGrowing, RegionGrowing, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, DepthImageTriangulation, cloud_algos::DepthImageTriangulation, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, BoxEstimation, cloud_algos::BoxEstimation, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, RobustBoxEstimation, cloud_algos::RobustBoxEstimation, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, CylinderEstimation, cloud_algos::CylinderEstimation, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, SVMClassification, cloud_algos::SVMClassification, cloud_algos::CloudAlgo);
//PLUGINLIB_DECLARE_CLASS(cloud_algos, GlobalRSD, cloud_algos::GlobalRSD, cloud_algos::CloudAlgo);
PLUGINLIB_DECLARE_CLASS(cloud_algos, StatisticalNoiseRemoval, cloud_algos::StatisticalNoiseRemoval, cloud_algos::CloudAlgo);

//PLUGINLIB_REGISTER_CLASS(NoiseRemoval, NoiseRemoval, PCD_Algo)

