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

class URDFRenderer
{ 
  public:
    URDFRenderer (std::string model_description, std::string tf_prefix);
    void initURDFModel ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (boost::shared_ptr<urdf::Link> link);
    void transforms_changed ();
    void render ();

  private:
    std::string model_description_;
    std::string tf_prefix_;
    double threshold_;
    std::vector<boost::shared_ptr<Renderable> > renderables_;
    tf::TransformListener tf_;
};

} // end namespace

#endif
