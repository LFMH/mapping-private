// to do the urdf / depth image intersection
#include <pthread.h>
#include <execinfo.h>

#include "realtime_perception/FrameBufferObject.h"

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
#include <cuda.h>
#include <cuda_gl_interop.h>

#include "opencv2/opencv.hpp"
#include "opencv2/gpu/gpu.hpp"

#include <iostream>

#include <ros/node_handle.h>
#include "realtime_perception/urdf_renderer.h"
#include "realtime_perception/point_types.h"
#include <realtime_perception/shader_wrapper.h>
#include <realtime_perception/urdf_filtering.h>

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
    KinectURDFSegmentation (ros::NodeHandle &nh, int argc, char **argv)
      : nh(nh)
      , normal_method(1)
      , nr_neighbors (36)
      , radius_cm (5)
      , normal_viz_step(200)
      , fbo_initialized_(false)
      , gl_image_needed (false)
      , gl_image_available (false)
      , argc (argc), argv(argv)
    {
      XmlRpc::XmlRpcValue v;
      nh.getParam ("fixed_frame", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "fixed_frame paramter!");
      fixed_frame_ = (std::string)v;
      ROS_INFO ("using fixed frame %s", fixed_frame_.c_str ());

      nh.getParam ("camera_frame", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeString && "need a camera_frame paramter!");
      cam_frame_ = (std::string)v;
      ROS_INFO ("using camera frame %s", cam_frame_.c_str ());

      nh.getParam ("camera_offset", v);
      ROS_ASSERT (v.getType() == XmlRpc::XmlRpcValue::TypeStruct && "need a camera_offset paramter!");
      ROS_ASSERT (v.hasMember ("translation") && v.hasMember ("rotation") && "camera offset needs a translation and rotation parameter!");

      XmlRpc::XmlRpcValue vec = v["translation"];
      ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 3 && "camera_offset.translation parameter must be a 3-value array!");
      ROS_INFO ("using camera translational offset: %f %f %f",
          (double)(vec[0]),
          (double)(vec[1]),
          (double)(vec[2])
          );
      camera_offset_t_ = tf::Vector3((double)vec[0], (double)vec[1], (double)vec[2]);

      vec = v["rotation"];
      ROS_ASSERT (vec.getType() == XmlRpc::XmlRpcValue::TypeArray && vec.size() == 4 && "camera_offset.rotation parameter must be a 4-value array [x y z w]!");
      ROS_INFO ("using camera rotational offset: %f %f %f %f", (double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);
      camera_offset_q_ = tf::Quaternion((double)vec[0], (double)vec[1], (double)vec[2], (double)vec[3]);

    }
  
    void 
      loadModels ()
    {
      XmlRpc::XmlRpcValue v;
      nh.getParam ("models", v);
      
      if (v.getType () == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < v.size(); ++i)
        {
          XmlRpc::XmlRpcValue elem = v[i];
          ROS_ASSERT (elem.getType()  == XmlRpc::XmlRpcValue::TypeStruct);

          std::string description_param = elem["model"];
          std::string tf_prefix = elem["tf_prefix"];

          // read URDF model
          std::string content;

          if (!nh.getParam(description_param, content))
          {
            std::string loc;
            if (nh.searchParam(description_param, loc))
            {
              nh.getParam(loc, content);
            }
            else
            {
              ROS_ERROR ("Parameter [%s] does not exist, and was not found by searchParam()",
                  description_param.c_str());
              continue;
            }
          }

          if (content.empty())
          {
            ROS_ERROR ("URDF is empty");
            continue;
          }

          // finally, set the model description so we can later parse it.
          renderers.push_back (new realtime_perception::URDFRenderer (content, tf_prefix, cam_frame_, fixed_frame_, tf_));
        }
      }
      else
      {
        ROS_ERROR ("models parameter must be an array!");
      }
    }

void print_backtrace ()
{
  void *array[100];
  size_t size;

  // get void*'s for all entries on the stack
  size = backtrace(array, 100);

  // print out all the frames to stderr
  backtrace_symbols_fd(array, size, 2);
}


    // callback function from the OpenNIGrabber
    template <template <typename> class Storage> void 
    cloud_cb (const boost::shared_ptr<openni_wrapper::Image>& image,
              const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image, 
              float constant)
    {
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

      // request a new gl image
      {
        boost::lock_guard<boost::mutex> lock(mutex_gl_image_needed);
        gl_image_needed=true;
      }
      std::cerr << "callback: notifying render () to run." << std::endl;
      cond_gl_image_needed.notify_all ();

      boost::unique_lock<boost::mutex> lock(mutex_gl_image_available);
      std::cerr << "callback: waiting for render () to finish." << std::endl;
      while (!gl_image_available)
        cond_gl_image_available.wait(lock);
      gl_image_available = false;
      std::cerr << "callback: render () finished." << std::endl;

      boost::mutex::scoped_lock gl_image_lock (gl_image_mutex);

      // PARAMETERS
      static int smoothing_nr_iterations = 10;
      static int smoothing_filter_size = 2;
      //static int enable_visualization = 0;
      //static int enable_mean_shift = 1;
      //static int enable_plane_fitting = 0;
      //static int meanshift_sp=8;
      //static int meanshift_sr=20;
      //static int meanshift_minsize=100;

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


      typename Storage<float>::type urdf_inliers;
      realtime_perception::BackgroundSubtraction bs;
      int num_urdf_inliers = bs.fromGLDepthImage<Storage> (depth_image, interop_cuda_pointer_depth_, constant, 0.5f, urdf_inliers, false, 1);
      std::cerr<< "NUM INLIERS: " << num_urdf_inliers << std::endl;

      typename ImageType<Storage>::type bla_image;
      ImageType<Storage>::createContinuous (480, 640, CV_8UC4, bla_image);
      typename StoragePointer<Storage,char4>::type bla_ptr = typename StoragePointer<Storage,char4>::type ((char4*)bla_image.data);
      thrust::copy (interop_cuda_pointer_normals_, interop_cuda_pointer_normals_ + 640 * 480, bla_ptr);

      float min, max;
      thrust::device_vector <float> vec (interop_cuda_pointer_depth_, interop_cuda_pointer_depth_ + 640 * 480);

      realtime_perception::find_extrema<Storage> (vec, min, max);
      std::cerr << "extrema " << min << ", " << max << std::endl;

      realtime_perception::find_extrema<Storage> (urdf_inliers, min, max);
      std::cerr << "extrema " << min << ", " << max << std::endl;


      cv::Mat gl_normal_image;
      cv::cvtColor(cv::Mat (bla_image), gl_normal_image, CV_BGRA2RGBA);
      cv::imshow ("URDF Models Normal Space", gl_normal_image);
      cv::waitKey (2);

      ImageType<Storage>::createContinuous (480, 640, CV_32FC1, bla_image);
      //thrust::copy (interop_cuda_pointer_depth_, interop_cuda_pointer_depth_ + 640 * 480, typename StoragePointer<Storage,float>::type ((float*)bla_image.data));
      thrust::copy (urdf_inliers.begin(), urdf_inliers.end(), typename StoragePointer<Storage,float>::type ((float*)bla_image.data));

      cv::imshow ("URDF Models Depth Component", cv::Mat (bla_image));
      cv::waitKey (2);

      //TODO: compute transformation between urdf model and kinect cloud.
    }

    template <typename T>
    void
      createPBO (GLuint &pbo, unsigned int size_tex_data, thrust::device_ptr<T> &cuda_pointer)
    {
      // create buffer object
      glGenBuffers (1, &pbo);
      glBindBuffer (GL_ARRAY_BUFFER, pbo);

      // buffer data
      glBufferData (GL_ARRAY_BUFFER, size_tex_data, NULL, GL_DYNAMIC_DRAW);
      glBindBuffer (GL_ARRAY_BUFFER, 0);

      // register buffer with cuda
      cudaGLRegisterBufferObject(pbo);
      T *raw_ptr = 0;
      cudaError_t cuda_error = cudaGLMapBufferObject((void**)&raw_ptr, pbo);
      cuda_pointer = thrust::device_pointer_cast(raw_ptr);
    }
  
    void initGL ()
    {
      //TODO: change this to use an offscreen pbuffer, so no window is necessary
      glutInit (&argc, argv);

      glutInitWindowSize (640, 480);
      glutInitWindowPosition(200, 100);
      glutInitDisplayMode ( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
      glutCreateWindow ("TestFramebufferObject");

      GLenum err = glewInit();
      if (GLEW_OK != err)
      {
        std::cout << "ERROR: could not initialize GLEW!" << std::endl;
      }

      interop_gl_buffers_.resize (2, GL_INVALID_VALUE);

      createPBO (interop_gl_buffers_[0], 640 * 480 * 4 * sizeof(GLubyte), interop_cuda_pointer_normals_);
      createPBO (interop_gl_buffers_[1], 640 * 480 * sizeof(GLfloat), interop_cuda_pointer_depth_);

      initFrameBufferObject ();
      loadModels ();
    }

    void initFrameBufferObject ()
    {
      fbo_ = new FramebufferObject ("rgba=4x8t depth=32t stencil=t");

      fbo_->initialize (640, 480);
      fbo_initialized_ = true;

      GLenum err = glGetError();
      if(err != GL_NO_ERROR)
        printf("OpenGL ERROR after FBO initialization: %s\n", gluErrorString(err));
    }

    void render ()
    {
      // lock gl_image_mutex and "produce" one image
      boost::mutex::scoped_lock gl_image_lock(gl_image_mutex);

      cudaError_t cuda_error = cudaGLUnmapBufferObject(interop_gl_buffers_[0]);
      cuda_error = cudaGLUnmapBufferObject(interop_gl_buffers_[1]);

      if (!fbo_initialized_)
        return;

      static const GLenum buffers[] = {
        GL_COLOR_ATTACHMENT0_EXT,
        GL_COLOR_ATTACHMENT1_EXT,
        GL_COLOR_ATTACHMENT2_EXT,
        GL_COLOR_ATTACHMENT3_EXT
      };

      tf::StampedTransform t;
      try
      {
        tf_.lookupTransform (cam_frame_, fixed_frame_, ros::Time (), t);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }

      // -----------------------------------------------------------------------
      // -----------------------------------------------------------------------
      //	1. render teapot into color, depth and stencil buffer

      GLenum err = glGetError();
      if(err != GL_NO_ERROR)
        printf("OpenGL ERROR after FBO initialization: %s\n", gluErrorString(err));
      else
        printf ("everything ok so far\n");

      glPushAttrib(GL_ALL_ATTRIB_BITS);
      glEnable(GL_NORMALIZE);
 
      fbo_->beginCapture();

      static realtime_perception::ShaderWrapper shader = realtime_perception::ShaderWrapper::fromFiles
        ("package://realtime_perception/include/shaders/test1.vert", 
         "package://realtime_perception/include/shaders/test1.frag");
      err = glGetError();
      if(err != GL_NO_ERROR)
        printf("before calling glUseProgram: OpenGL ERROR: %s\n", gluErrorString(err));

      shader ();
      err = glGetError();
      if(err != GL_NO_ERROR)
        printf("after calling glUseProgram: OpenGL ERROR: %s\n", gluErrorString(err));

      glDrawBuffers(sizeof(buffers) / sizeof(GLenum), buffers);

      // clear the buffers
      glClearColor(0.0, 0.0, 0.0, 1.0);
      glClearStencil(0x0);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_ALWAYS, 0x1, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

      glEnable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      // setup camera
      glMatrixMode (GL_PROJECTION);
      glLoadIdentity();
      float near_clip = 0.1;
      float far_clip = 100;
      float width = 640;
      float height = 480;
      float fx = 525; // P[0]
      float fy = 525; // P[5]
      float cx = 319.5; // P[2]
      float cy = 239.5; // P[6]
      float f_width = near_clip * width / fx;
      float f_height = near_clip * height / fy;

      //note: we swap left and right frustrum to swap handedness of ROS vs. OpenGL
      glFrustum (  f_width  * (1.0f - cx / width),
                 - f_width  *         cx / width,
                 - f_height * (1.0f - cy / height),
                   f_height *         cy / height,
                 near_clip,
                 far_clip);
      glMatrixMode(GL_MODELVIEW);
      glLoadIdentity();
      gluLookAt (0,0,0, 0,0,1, 0,1,0);

      tf::Transform transform (camera_offset_q_, camera_offset_t_);
      btScalar glTf[16];
      transform.getOpenGLMatrix(glTf);
      glMultMatrixd((GLdouble*)glTf);

      t.getOpenGLMatrix(glTf);
      glMultMatrixd((GLdouble*)glTf);

      {
        ScopeTimeCPU berst("TF lookup and rendering");
        BOOST_FOREACH (realtime_perception::URDFRenderer* r, renderers)
          r->render ();
      }

      glUseProgram(NULL);
      
      fbo_->endCapture();

      glPopAttrib();

      // -----------------------------------------------------------------------
      // -----------------------------------------------------------------------
      //	2. render red plane only where stencil is 1

      glPushAttrib(GL_ALL_ATTRIB_BITS);

      fbo_->beginCapture();

      glDrawBuffer(GL_COLOR_ATTACHMENT3_EXT);

      glEnable(GL_STENCIL_TEST);
      glStencilFunc(GL_EQUAL, 0x1, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      glColor3f(1.0, 0.0, 0.0);

      glBegin(GL_QUADS);
        glVertex2f(0.0, 0.0);
        glVertex2f(1.0, 0.0);
        glVertex2f(1.0, 1.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();

      glStencilFunc(GL_EQUAL, 0x0, 0x1);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

      glDisable(GL_DEPTH_TEST);
      glDisable(GL_TEXTURE_2D);
      fbo_->disableTextureTarget();

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      glColor3f(0.0, 0.0, 1.0);

      glBegin(GL_QUADS);
        glVertex2f(0.0, 0.0);
        glVertex2f(1.0, 0.0);
        glVertex2f(1.0, 1.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      
      fbo_->endCapture();

      glPopAttrib();

      // TODO: code to render the offscreen buffer
      // -----------------------------------------------------------------------
      // -----------------------------------------------------------------------

      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

      glMatrixMode(GL_PROJECTION);
      glPushMatrix();
      glLoadIdentity();
      gluOrtho2D(0.0, 1.0, 0.0, 1.0);

      glMatrixMode(GL_MODELVIEW);	
      glPushMatrix();
      glLoadIdentity();

      // draw color buffer 0
      fbo_->bind(0);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.0, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.333, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.333, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 1.0);
      glEnd();

      // draw color buffer 1
      fbo_->bind(1);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.0, 0.0);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.333, 0.0);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.333, 0.5);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.0, 0.5);
      glEnd();

      // draw color buffer 2
      fbo_->bind(2);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.333, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.666, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.666, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.333, 1.0);
      glEnd();

      // draw color buffer 3
      fbo_->bind(3);
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.333, 0.0);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(0.666, 0.0);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(0.666, 0.5);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.333, 0.5);
      glEnd();

      // draw depth buffer 
      fbo_->bindDepth();
      glBegin(GL_QUADS);
        glTexCoord2f(0.0, fbo_->getHeight());
        glVertex2f(0.666, 0.5);
        glTexCoord2f(fbo_->getWidth(), fbo_->getHeight());
        glVertex2f(1.0, 0.5);
        glTexCoord2f(fbo_->getWidth(), 0.0);
        glVertex2f(1.0, 1.0);
        glTexCoord2f(0.0, 0.0);
        glVertex2f(0.666, 1.0);
      glEnd();

      glPopMatrix();
      glMatrixMode(GL_PROJECTION);
      glPopMatrix();
      
      // copy frame buffer attachments to pbo's
      fbo_->bind (2);
      glBindBuffer (GL_PIXEL_PACK_BUFFER, interop_gl_buffers_[0]);
      glGetTexImage (fbo_->getTextureTarget(), 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

      // copy frame buffer attachments to pbo's
      fbo_->bindDepth ();
      glBindBuffer (GL_PIXEL_PACK_BUFFER, interop_gl_buffers_[1]);
      glGetTexImage (fbo_->getTextureTarget(), 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
      glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);

      // get device pointer to buffer
      char4 *raw_ptr = 0;
      cuda_error = cudaGLMapBufferObject((void**)&raw_ptr, interop_gl_buffers_[0]);
      interop_cuda_pointer_normals_ = thrust::device_pointer_cast(raw_ptr);

      // get device pointer to buffer
      float *raw_ptr_d = 0;
      cuda_error = cudaGLMapBufferObject((void**)&raw_ptr_d, interop_gl_buffers_[1]);
      interop_cuda_pointer_depth_ = thrust::device_pointer_cast(raw_ptr_d);
     
      {
        boost::lock_guard<boost::mutex> lock(mutex_gl_image_available);
        gl_image_available=true;
      }

      glutSwapBuffers ();
      glutPostRedisplay();
      glutMainLoopEvent ();

      // make sure that depth_range is [0;1]
      GLfloat depth_range[2];
      glGetFloatv (GL_DEPTH_RANGE, &depth_range[0]);

      std::cerr << "render: notify callback. " << depth_range[0] << " -- " << depth_range[1] << std::endl;
      cond_gl_image_available.notify_all ();
    }
    
    void 
    run ()
    {
      initGL ();

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
        // wait for "data needed!"
        boost::unique_lock<boost::mutex> lock (mutex_gl_image_needed);
        while (!gl_image_needed && nh.ok ())
          cond_gl_image_needed.wait(lock);
        
        render();
        gl_image_needed = false;

        glutPostRedisplay();
        glutMainLoopEvent ();
      }

      grabber->stop ();
    }

    ros::NodeHandle &nh;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normal_cloud;
    pcl::PointCloud<PointXYZRGBNormalRegion>::Ptr region_cloud;
    DisparityToCloud d2c;
    pcl::visualization::CloudViewer *viewer;

    int normal_method;
    int nr_neighbors;
    int radius_cm;
    int normal_viz_step;
    std::vector<realtime_perception::URDFRenderer*> renderers;

    tf::Vector3 camera_offset_t_;
    tf::Quaternion camera_offset_q_;
    std::string cam_frame_;
    std::string fixed_frame_;
    FramebufferObject *fbo_;
    bool fbo_initialized_;
    tf::TransformListener tf_;

    // variables holding the CUDA/OpenGL interop buffer info
    std::vector<GLuint> interop_gl_buffers_;
    thrust::device_ptr<float> interop_cuda_pointer_depth_;
    thrust::device_ptr<char4> interop_cuda_pointer_normals_;

    // these are needed for the synchronous across-thread-boundaries call of render() from cloud_cb()
    bool gl_image_needed;
    bool gl_image_available;
    boost::mutex mutex_gl_image_needed;
    boost::mutex mutex_gl_image_available;
    boost::condition_variable cond_gl_image_needed;
    boost::condition_variable cond_gl_image_available;
    boost::mutex gl_image_mutex;

    int argc;
    char **argv;
};

int 
main (int argc, char **argv)
{
  ros::init (argc, argv, "urdf_filter_cuda");

  ros::NodeHandle nh ("~");

  KinectURDFSegmentation s(nh, argc, argv);
  s.run ();

  return 0;
}

