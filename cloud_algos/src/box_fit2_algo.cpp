/* 
 * Copyright (c) 2010, Zoltan-Csaba Marton <marton@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cloud_algos/box_fit2_algo.h>

// Sample Consensus
#include <ias_sample_consensus/sac_model_orientation.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>

// For computeCentroid
#include <point_cloud_mapping/geometry/nearest.h>

using namespace std;
using namespace cloud_algos;
using namespace ias_sample_consensus;
using namespace sample_consensus;

std::vector<std::string> RobustBoxEstimation::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  // requires normals
  requires.push_back("nx");
  requires.push_back("ny");
  requires.push_back("nz");
  return requires;
}

void RobustBoxEstimation::pre () : BoxEstimation::pre ()
{
  nh_.param("eps_angle", eps_angle_, eps_angle_);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */
void RobustBoxEstimation::find_model(boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff)
{
  // Compute center point
  cloud_geometry::nearest::computeCentroid (*cloud, box_centroid_);
  coeff[0] = box_centroid_.x;
  coeff[1] = box_centroid_.y;
  coeff[2] = box_centroid_.z;

  // Create model
  SACModelOrientation *model = new SACModelOrientation ();
  model->axis_ << 0, 0, 1; // Suppose fixed Z axis
  model->setDataSet (&cloud); // TODO: this is nasty :)

  // Fit model using RANSAC
  RANSAC *sac = new RANSAC (model, eps_angle_);
  vector<int> inliers;
  vector<double> refined;
  if (!sac->computeModel ())
  {
    ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %d!", eps_angle_);
    return;
  }

  // Get inliers and refine result
  inliers = sac->getInliers ();
  /// @NOTE best_model_ contains actually the samples used to find the best model!
  vector<double> original = model_->getModelCoefficients ();
  cerr << "original direction: " << original.at (0) << " " << original.at (1) << " " << original.at (2) << ", found at point nr " << original.at (3) << endl;
  sac->refineCoefficients(refined);
  cerr << "refined direction: " << refined.at (0) << " " << refined.at (1) << " " << refined.at (2) << ", initiated from point nr " << refined.at (3) << endl;
  if (refined[3] == -1)
    refined = original;

  // Save dimensions
  coeff[3+0] = 0.0;
  coeff[3+1] = 0.0;
  coeff[3+2] = 0.0;

  // Save principle axis
  coeff[6+0] = refined[0];
  coeff[6+1] = refined[1];
  coeff[6+2] = refined[2];

  // Save fixed axis
  coeff[12+0] = model->axis_[0];
  coeff[12+1] = model->axis_[1];
  coeff[12+2] = model->axis_[2];

  // Save complementary axis (cross product)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];

  // Print info
  ROS_INFO ("[RobustBoxEstimation] Center x: %f, y: %f, z: %f",
      coeff[0], coeff[1], coeff[2]);
  ROS_INFO ("[RobustBoxEstimation] Dimensions x: %f, y: %f, z: %f",
      coeff[3+0], coeff[3+1], coeff[3+2]);
  ROS_INFO ("[RobustBoxEstimation] Direction vectors: \n\t%f %f %f \n\t%f %f %f \n\t%f %f %f",
      coeff[3+3], coeff[3+4], coeff[3+5],
      coeff[3+6], coeff[3+7], coeff[3+8],
      coeff[3+9], coeff[3+10],coeff[3+11]);
}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RobustBoxEstimation> (argc, argv);
}
#endif

