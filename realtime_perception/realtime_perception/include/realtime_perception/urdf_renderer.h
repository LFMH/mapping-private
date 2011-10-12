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
    URDFRenderer (boost::shared_ptr<OffscreenRenderer> canvas,
                  std::string model_description, std::string tf_prefix);
    ~URDFRenderer ();
    void initURDFModel ();
    void loadURDFModel (urdf::Model &descr);
    void process_link (boost::shared_ptr<urdf::Link> link);
    void transforms_changed ();
    void render ();

  private:
    boost::shared_ptr<OffscreenRenderer> canvas;
    std::string model_description_;
    std::string tf_prefix_;
    double threshold_;
    std::vector<boost::shared_ptr<Renderable> > renderables_;
    tf::TransformListener tf_;
};

} // end namespace

#endif
