#include <pluginlib/class_list_macros.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/normal_estimation.h>
#include <cloud_algos/planar_estimation.h>
#include <cloud_algos/rotational_estimation.h>
#include <cloud_algos/region_growing.h>
#include <cloud_algos/box_fit_algo.h>
#include <cloud_algos/depth_image_triangulation.h>
#include <cloud_algos/mls_fit.h>

using namespace cloud_algos;

//register NormalEstimation as a PCD_Algo class
//PLUGINLIB_REGISTER_CLASS(NormalEstimation, NormalEstimation, PCD_Algo)
//PLUGINLIB_REGISTER_CLASS(PlanarEstimation, PlanarEstimation, PCD_Algo)
PLUGINLIB_REGISTER_CLASS(MovingLeastSquares, MovingLeastSquares, CloudAlgo)
PLUGINLIB_REGISTER_CLASS(RotationalEstimation, RotationalEstimation, CloudAlgo)
PLUGINLIB_REGISTER_CLASS(RegionGrowing, RegionGrowing, CloudAlgo)
PLUGINLIB_REGISTER_CLASS(DepthImageTriangulation, DepthImageTriangulation, CloudAlgo)
PLUGINLIB_REGISTER_CLASS(BoxEstimation, BoxEstimation, CloudAlgo)
//PLUGINLIB_REGISTER_CLASS(NoiseRemoval, NoiseRemoval, PCD_Algo)

