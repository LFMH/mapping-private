#ifndef REALTIME_PERCEPTION_RENDERABLE_H_
#define REALTIME_PERCEPTION_RENDERABLE_H_

#include <GL/gl.h>
#include <tf/tf.h>
#include <urdf/color.h>

// forward declare
struct aiScene;
struct aiMesh;

namespace realtime_perception
{

struct Renderable
{ 
  void setLinkName (std::string n);
  virtual void render () = 0;
  std::string name;
  tf::Vector3 offset_t;
  tf::Quaternion offset_q;
  tf::Vector3 t;
  tf::Quaternion q;
  urdf::Color color;
  void applyTransform ();
  void unapplyTransform ();
};

struct RenderableBox : public Renderable
{
  RenderableBox (float dimx, float dimy, float dimz);

  virtual void render ();
    
  float dimx, dimy, dimz;
};

struct RenderableSphere : public Renderable
{
  RenderableSphere (float radius);

  virtual void render ();

  float radius;
};

// meshes are a tad more complicated than boxes and spheres
struct RenderableMesh : public Renderable
{
  RenderableMesh (std::string meshname);

  virtual void render ();

private:
  struct Vertex
  {
    float x,y,z;
    float nx,ny,nz;
    Vertex (float x, float y, float z, float nx, float ny, float nz)
      : x(x), y(y), z(z), nx(nx), ny(ny), nz(nz)
    {}
  };

  struct SubMesh
  {
    enum {INVALID_VALUE = 0xFFFFFFFF};
    SubMesh ();
    ~SubMesh ();
    void init (const std::vector<Vertex>& vertices,
               const std::vector<unsigned int>& indices);
    GLuint vbo, ibo;
    unsigned int num_indices;
  };

  void fromAssimpScene (const aiScene* scene);
  void initMesh (unsigned int i, const aiMesh* mesh);

  std::vector<SubMesh> meshes;
};



} // end namespace

#endif

