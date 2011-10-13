#ifndef REALTIME_PERCEPTION_URDF_RENDERER_H_
#define REALTIME_PERCEPTION_URDF_RENDERER_H_

#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <realtime_perception/renderable.h>

// forward declares
namespace ros {class NodeHandle;}

namespace realtime_perception
{

class URDFRenderer
{ 
  public:
    URDFRenderer (std::string model_description, std::string tf_prefix, std::string cam_frame);
    void initURDFModel ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (boost::shared_ptr<urdf::Link> link);
    void transforms_changed ();
    void render ();
    void setCameraFrame (std::string &cam_f);

  private:
    // urdf model stuff
    std::string model_description_;
    std::string tf_prefix_;
    
    // rendering stuff 
    std::vector<boost::shared_ptr<Renderable> > renderables_;
    tf::TransformListener tf_;

    // camera stuff
    std::string camera_frame_;
};

} // end namespace

#endif
