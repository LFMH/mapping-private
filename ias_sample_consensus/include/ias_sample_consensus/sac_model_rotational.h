#ifndef IAS_SAMPLE_CONSENSUS_SACMODELROTATIONAL_H_
#define IAS_SAMPLE_CONSENSUS_SACMODELROTATIONAL_H_

#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <ias_table_msgs/TriangularMesh.h>
#include <mapping_msgs/PolygonalMap.h>

namespace ias_sample_consensus
{
  /** \brief A Sample Consensus Model class for 3D plane segmentation.
    */
  class SACModelRotational : public sample_consensus::SACModel
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for SACModelRotational. */
      SACModelRotational (boost::shared_ptr<mapping_msgs::PolygonalMap> pmap) : pmap_(pmap) 
      { 
        nx_idx_ = ny_idx_ = nz_idx_ = -1;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for SACModelRotational. */
      virtual ~SACModelRotational () { }

      virtual void getSamples (int &iterations, std::vector<int> &samples);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Test whether the given model coefficients are valid given the input point cloud data.
        * \param model_coefficients the model coefficients that need to be tested
        * \todo implement this
        */
      bool testModelCoefficients (const std::vector<double> &model_coefficients) { return true; }

      virtual bool computeModelCoefficients (const std::vector<int> &samples);

      virtual void refitModel (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      virtual void projectPoints (const std::vector<int> &inliers, const std::vector<double> &model_coefficients, sensor_msgs::PointCloud &projected_points);

      virtual void projectPointsInPlace (const std::vector<int> &inliers, const std::vector<double> &model_coefficients);
      virtual bool doSamplesVerifyModel (const std::set<int> &indices, double threshold);

      static int functionToOptimize (void *p, int m, int n, const double *x, double *fvec, int iflag);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (666 for now). */
      virtual int getModelType () { return (666); }
      
      void samplePointsOnRotational (const std::vector<double> modelCoefficients, std::vector<int> inliers, boost::shared_ptr<ias_table_msgs::TriangularMesh>);
      static int functionToOptimizeAxis (void *p, int m, int n, const double *x, double *fvec, int iflag);
      bool MinimizeAxisDistancesToSamples (const std::vector<int> samples, std::vector<double> &model_coefficients, double &err);
      double PointToRotationalDistance (const std::vector<double> &model_coefficients, const geometry_msgs::Point32 &p);
      bool EstimateAxisFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients);
      bool EstimateContourFromSamples (const std::vector<int> samples, std::vector<double> &model_coefficients);
      bool RefitAxis (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      void RefitContour (const std::vector<int> &inliers, std::vector<double> &refit_coefficients);
      boost::shared_ptr<mapping_msgs::PolygonalMap> pmap_;
      int nx_idx_;
      int ny_idx_;
      int nz_idx_;
      const std::vector<int> *tmp_inliers_;
  };
}

#endif
