// to do the urdf / depth image intersection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/win32_macros.h>

#include <boost/shared_ptr.hpp>

#include <pcl/cuda/features/normal_3d.h>
#include <pcl/cuda/time_cpu.h>
#include <pcl/cuda/time_gpu.h>
#include <pcl/cuda/io/cloud_to_pcl.h>
#include <pcl/cuda/io/extract_indices.h>
#include <pcl/cuda/io/disparity_to_cloud.h>
#include <pcl/cuda/io/host_device.h>
#include "pcl/cuda/sample_consensus/sac_model_1point_plane.h"
#include "pcl/cuda/sample_consensus/multi_ransac.h"
#include <pcl/cuda/segmentation/connected_components.h>
#include <pcl/cuda/segmentation/mssegmentation.h>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include <iostream>

#include <ros/node_handle.h>
#include "realtime_perception/urdf_renderer.h"
#include "realtime_perception/point_types.h"
#include <GL/freeglut.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace pcl::cuda;

template <template <typename> class Storage>
struct ImageType
{
  typedef void type;
};

template <>
struct ImageType<Device>
{
  typedef cv::gpu::GpuMat type;
  static void createContinuous (int h, int w, int typ, type &mat)
  {
    cv::gpu::createContinuous (h, w, typ, mat);
  }
};

template <>
struct ImageType<Host>
{
  typedef cv::Mat type;
  static void createContinuous (int h, int w, int typ, type &mat)
  {
    mat = cv::Mat (h, w, typ); // assume no padding at the end of line
  }
};

class KinectURDFSegmentation
{
  public:
    KinectURDFSegmentation (ros::NodeHandle &nh)
      : nh(nh)
      , new_cloud(false)
      , normal_method(1)
      , nr_neighbors (36)
      , radius_cm (5)
      , normal_viz_step(200)
    {
      renderer = new realtime_perception::URDFRenderer (nh);
    }

    // callback function from the OpenNIGrabber
    template <template <typename> class Storage> void 
    cloud_cb (const boost::shared_ptr<openni_wrapper::Image>& image,
              const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, 
              float constant)
    {
      ROS_ERROR ("-");
      // TIMING
      static unsigned count = 0;
      static double last = getTime ();
      double now = getTime ();

      //if (++count == 30 || (now - last) > 5)
      {
        std::cout << std::endl;
        count = 1;
        std::cout << "Average framerate: " << double(count)/double(now - last) << " Hz --- ";
        last = now;
      }

      // PARAMETERS
      static int smoothing_nr_iterations = 10;
      static int smoothing_filter_size = 2;
      static int enable_visualization = 0;
      static int enable_mean_shift = 1;
      static int enable_plane_fitting = 0;
      static int meanshift_sp=8;
      static int meanshift_sr=20;
      static int meanshift_minsize=100;

      // CPU AND GPU POINTCLOUD OBJECTS
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
      typename PointCloudAOS<Storage>::Ptr data;

      ScopeTimeCPU time ("everything");
      {
        ScopeTimeCPU time ("disparity smoothing");
        // Compute the PointCloud on the device
        d2c.compute<Storage> (depth_image, image, constant, data, false, 1, smoothing_nr_iterations, smoothing_filter_size);
      }

      // GPU NORMAL CLOUD
      boost::shared_ptr<typename Storage<float4>::type> normals;
      float focallength = 580/2.0;
      {
        ScopeTimeCPU time ("Normal Estimation");
        if (normal_method == 1)
          // NORMAL ESTIMATION USING DIRECT PIXEL NEIGHBORS
          normals = computeFastPointNormals<Storage> (data);
        else
          // NORMAL ESTIMATION USING NEIGHBORHOODS OF RADIUS "radius"
          normals = computePointNormals<Storage> (data->points.begin (), data->points.end (), focallength, data, radius_cm / 100.0f, nr_neighbors);
        cudaThreadSynchronize ();
      }

      // RETRIEVE NORMALS AS AN IMAGE
      typename ImageType<Storage>::type normal_image;
      typename StoragePointer<Storage,char4>::type ptr;
      {
        ScopeTimeCPU time ("Matrix Creation");
        ImageType<Storage>::createContinuous ((int)data->height, (int)data->width, CV_8UC4, normal_image);
        ptr = typename StoragePointer<Storage,char4>::type ((char4*)normal_image.data);
        createNormalsImage<Storage> (ptr, *normals);
      }

      //urdf_filter->filter (data);

      //TODO: this breaks for pcl::cuda::Host, meaning we have to run this on the GPU
      std::vector<int> reg_labels;
      pcl::cuda::detail::DjSets comps(0);
      cv::Mat seg;
      {
        ScopeTimeCPU time ("Mean Shift");
        if (enable_mean_shift == 1)
        {
          // USE GPU MEAN SHIFT SEGMENTATION FROM OPENCV
          pcl::cuda::meanShiftSegmentation (normal_image, seg, reg_labels, meanshift_sp, meanshift_sr, meanshift_minsize, comps);
          typename Storage<char4>::type new_colors ((char4*)seg.datastart, (char4*)seg.dataend);
          colorCloud<Storage> (data, new_colors);
        }
      }

      typename SampleConsensusModel1PointPlane<Storage>::Ptr sac_model;
      if (enable_plane_fitting == 1)
      {
        // Create sac_model
        {
          ScopeTimeCPU t ("creating sac_model");
          sac_model.reset (new SampleConsensusModel1PointPlane<Storage> (data));
        }
        sac_model->setNormals (normals);

        MultiRandomSampleConsensus<Storage> sac (sac_model);
        sac.setMinimumCoverage (0.90); // at least 95% points should be explained by planes
        sac.setMaximumBatches (1);
        sac.setIerationsPerBatch (1024);
        sac.setDistanceThreshold (0.05);

        {
          ScopeTimeCPU timer ("computeModel: ");
          if (!sac.computeModel (0))
          {
            std::cerr << "Failed to compute model" << std::endl;
          }
          else
          {
            if (enable_visualization)
            {
//              std::cerr << "getting inliers.. ";
              
              std::vector<typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr> planes;
              typename Storage<int>::type region_mask;
              markInliers<Storage> (data, region_mask, planes);
              thrust::host_vector<int> regions_host;
              std::copy (regions_host.begin (), regions_host.end(), std::ostream_iterator<int>(std::cerr, " "));
              {
                ScopeTimeCPU t ("retrieving inliers");
                planes = sac.getAllInliers ();
              }
              std::vector<int> planes_inlier_counts = sac.getAllInlierCounts ();
              std::vector<float4> coeffs = sac.getAllModelCoefficients ();
              std::vector<float3> centroids = sac.getAllModelCentroids ();
              std::cerr << "Found " << planes_inlier_counts.size () << " planes" << std::endl;
              int best_plane = 0;
              int best_plane_inliers_count = -1;

              for (unsigned int i = 0; i < planes.size (); i++)
              {
                if (planes_inlier_counts[i] > best_plane_inliers_count)
                {
                  best_plane = i;
                  best_plane_inliers_count = planes_inlier_counts[i];
                }

                typename SampleConsensusModel1PointPlane<Storage>::IndicesPtr inliers_stencil;
                inliers_stencil = planes[i];//sac.getInliersStencil ();

                OpenNIRGB color;
                //double trand = 255 / (RAND_MAX + 1.0);

                //color.r = (int)(rand () * trand);
                //color.g = (int)(rand () * trand);
                //color.b = (int)(rand () * trand);
                color.r = (1.0f + coeffs[i].x) * 128;
                color.g = (1.0f + coeffs[i].y) * 128;
                color.b = (1.0f + coeffs[i].z) * 128;
                {
                  ScopeTimeCPU t ("coloring planes");
                  colorIndices<Storage> (data, inliers_stencil, color);
                }
              }
            }
          }
        }
      }

      //else
      //{
      //  {
      //    ScopeTimeCPU c_p ("Copying");
      //    boost::mutex::scoped_lock l(m_mutex);
      //    region_cloud.reset (new pcl::PointCloud<PointXYZRGBNormalRegion>);
      //    region_cloud->header.frame_id = "/openni_rgb_optical_frame";
      //    toPCL (*data, *normals, *region_cloud);
      //    if (enable_mean_shift)
      //    {
      //      for (int cp = 0; cp < region_cloud->points.size (); cp++)
      //      {
      //        region_cloud->points[cp].region = reg_labels[cp];
      //      }
      //    }
      //    new_cloud = true;
      //  }
      //  ScopeTimeCPU c_p ("Publishing");
      //  publish_cloud ();
      //}
    }
    
    void 
    run ()
    {
      pcl::Grabber* grabber = new pcl::OpenNIGrabber();

      boost::signals2::connection c;
      if (true)
      {
        std::cerr << "[KinectURDFSegmentation] Using GPU..." << std::endl;
        boost::function<void (const boost::shared_ptr<openni_wrapper::Image>& image, const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, float)> f = boost::bind (&KinectURDFSegmentation::cloud_cb<Device>, this, _1, _2, _3);
        c = grabber->registerCallback (f);
      }

      grabber->start ();
      
      while (nh.ok())
      {
        //ROS_INFO (".");
        renderer->render ();
        glutMainLoopEvent ();
        //pcl_sleep (1);
      }

      grabber->stop ();
    }

    ros::NodeHandle &nh;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    pcl::PointCloud<PointXYZRGBNormalRegion>::Ptr region_cloud;
    DisparityToCloud d2c;
    pcl::visualization::CloudViewer *viewer;
    boost::mutex m_mutex;
    bool new_cloud;
    int normal_method;
    int nr_neighbors;
    int radius_cm;
    int normal_viz_step;
    //std::auto_ptr <realtime_perception::URDFCloudFilter<pcl::PointXYZRGB> > urdf_filter;
    realtime_perception::URDFRenderer* renderer;
};



int 
main (int argc, char **argv)
{
  glutInit (&argc, argv);
  ros::init (argc, argv, "urdf_filter_cuda");
  
  ros::NodeHandle nh ("~");

  KinectURDFSegmentation s(nh);
  s.run ();

  return 0;
}


