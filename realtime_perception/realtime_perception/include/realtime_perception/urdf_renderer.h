#ifndef REALTIME_PERCEPTION_URDF_RENDERER_H_
#define REALTIME_PERCEPTION_URDF_RENDERER_H_

#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <GL/gl.h>
#include <realtime_perception/renderable.h>

// forward declares
namespace ros {class NodeHandle;}

namespace realtime_perception
{
class OffscreenRenderer;

class URDFRenderer
{ 
  public:
    URDFRenderer (ros::NodeHandle &nh);
    ~URDFRenderer ();
    void initURDFModel ();
    void loadParams ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (boost::shared_ptr<urdf::Link> link);
    void transforms_changed ();
    void render ();

  private:
    ros::NodeHandle &nh;
    std::string model_description_;
    std::string description_param_;
    std::string tf_prefix_;
    double threshold_;
    std::vector<Renderable*> renderables_;
    std::auto_ptr<OffscreenRenderer> canvas;
    tf::TransformListener tf_;
};

} // end namespace

#endif
