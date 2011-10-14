#define GL3_PROTOTYPES 1
#include <GL3/gl3.h>
#include <GL/freeglut.h>
#include <realtime_perception/renderable.h>
#include <resource_retriever/retriever.h>
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>

namespace realtime_perception
{
  // common methods
  void Renderable::setLinkName (std::string n)
  {
    name = n;
  }

  void Renderable::applyTransform ()
  {
    glPushMatrix ();

    tf::Transform transform (q,t);
    transform *= tf::Transform (offset_q, offset_t);
    btScalar glTf[16];
    transform.getOpenGLMatrix(glTf);
    glMultMatrixd((GLdouble*)glTf);
  }

  void Renderable::unapplyTransform ()
  {
    glPopMatrix ();
  }

  // Sphere methods
  RenderableSphere::RenderableSphere (float radius)
    : radius(radius)
  {}

  void RenderableSphere::render ()
  {
    applyTransform ();
    glutSolidSphere(radius, 10, 10);
    unapplyTransform ();
  }

  // Cylinder methods
  RenderableCylinder::RenderableCylinder (float radius, float length)
    : radius(radius), length(length)
  {}

  void RenderableCylinder::render ()
  {
    applyTransform ();
    glTranslatef (0, 0, -length/2);
    glutSolidCylinder(radius, length, 10, 10);
    unapplyTransform ();
  }

  // Box methods
  RenderableBox::RenderableBox (float dimx, float dimy, float dimz)
    : dimx(dimx), dimy(dimy), dimz(dimz)
  {}

  void RenderableBox::render ()
  {
    applyTransform ();

    glColor3f (color.r, color.g, color.b);
    glScalef (dimx, dimy, dimz);
    glutSolidCube (1.0);
    unapplyTransform ();
  }

  // these classes are copied from RVIZ. TODO: header/license/author tags
  class ResourceIOStream : public Assimp::IOStream
  {
  public:
    ResourceIOStream(const resource_retriever::MemoryResource& res)
    : res_(res)
    , pos_(res.data.get())
    {}

    ~ResourceIOStream()
    {}

    size_t Read(void* buffer, size_t size, size_t count)
    {
      size_t to_read = size * count;
      if (pos_ + to_read > res_.data.get() + res_.size)
      {
        to_read = res_.size - (pos_ - res_.data.get());
      }

      memcpy(buffer, pos_, to_read);
      pos_ += to_read;

      return to_read;
    }

    size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

    aiReturn Seek( size_t offset, aiOrigin origin)
    {
      uint8_t* new_pos = 0;
      switch (origin)
      {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
        break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
        break;
      default:
        ROS_BREAK();
      }

      if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
      {
        return aiReturn_FAILURE;
      }

      pos_ = new_pos;
      return aiReturn_SUCCESS;
    }

    size_t Tell() const
    {
      return pos_ - res_.data.get();
    }

    size_t FileSize() const
    {
      return res_.size;
    }

    void Flush() {}

  private:
    resource_retriever::MemoryResource res_;
    uint8_t* pos_;
  };

  class ResourceIOSystem : public Assimp::IOSystem
  {
  public:
    ResourceIOSystem()
    {
    }

    ~ResourceIOSystem()
    {
    }

    // Check whether a specific file exists
    bool Exists(const char* file) const
    {
      // Ugly -- two retrievals where there should be one (Exists + Open)
      // resource_retriever needs a way of checking for existence
      // TODO: cache this
      resource_retriever::MemoryResource res;
      try 
      {   
        res = retriever_.get(file);
      }   
      catch (resource_retriever::Exception& e)
      {   
        return false;
      }   

      return true;
    }

    // Get the path delimiter character we'd like to see
    char getOsSeparator() const
    {
      return '/';
    }

    // ... and finally a method to open a custom stream
    Assimp::IOStream* Open(const char* file, const char* mode)
    {
      ROS_ASSERT(mode == std::string("r") || mode == std::string("rb"));

      // Ugly -- two retrievals where there should be one (Exists + Open)
      // resource_retriever needs a way of checking for existence
      resource_retriever::MemoryResource res;
      try 
      {   
        res = retriever_.get(file);
      }   
      catch (resource_retriever::Exception& e)
      {   
        return 0;
      }   

      return new ResourceIOStream(res);
    }

    void Close(Assimp::IOStream* stream) { delete stream; }

  private:
    mutable resource_retriever::Retriever retriever_;
  };

  RenderableMesh::RenderableMesh (std::string meshname)
  {
    Assimp::Importer importer;
    importer.SetIOHandler(new ResourceIOSystem());
    const aiScene* scene = importer.ReadFile(meshname, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
    if (!scene)
    {
      ROS_ERROR("Could not load resource [%s]: %s", meshname.c_str(), importer.GetErrorString());
      return;
    }

    fromAssimpScene(scene);
  }

  RenderableMesh::SubMesh::SubMesh ()
  {
    vbo = INVALID_VALUE;
    ibo = INVALID_VALUE;
    num_indices = 0;
  }

  RenderableMesh::SubMesh::~SubMesh ()
  {
    if (vbo != INVALID_VALUE)
      glDeleteBuffers (1, &vbo);
    if (ibo != INVALID_VALUE)
      glDeleteBuffers (1, &ibo);
  }

  void RenderableMesh::SubMesh::init (const std::vector<Vertex>& vertices,
                                      const std::vector<unsigned int>& indices)
  {
    num_indices = indices.size ();
    glGenBuffers (1, &vbo);
    glBindBuffer (GL_ARRAY_BUFFER, vbo);
    glBufferData (GL_ARRAY_BUFFER, sizeof(Vertex) * vertices.size (), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers (1, &ibo);
    glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData (GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * num_indices, &indices[0], GL_STATIC_DRAW);
  }

  void RenderableMesh::fromAssimpScene (const aiScene* scene)
  {
    meshes.resize (scene->mNumMeshes);
    for (unsigned int i = 0; i < meshes.size (); ++i)
    {
      const aiMesh* mesh = scene->mMeshes[i];
      initMesh (i, mesh);
    }
  }

  void RenderableMesh::initMesh (unsigned int index, const aiMesh* mesh)
  {
    // TODO: mesh->mMaterialIndex
    // TODO: const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
    {
      const aiVector3D* pos = &(mesh->mVertices[i]);
      const aiVector3D* n = &(mesh->mNormals[i]);
      // TODO: const aiVector3D* pTexCoord = mesh->HasTextureCoords(0) ? &(mesh->mTextureCoords[0][i]) : &Zero3D;

      Vertex v(pos->x, pos->y, pos->z, n->x, n->y, n->z);
      vertices.push_back (v);
    }

    for (unsigned int i = 0 ; i < mesh->mNumFaces ; ++i)
    {
        const aiFace& face = mesh->mFaces[i];
        assert(face.mNumIndices == 3);
        indices.push_back(face.mIndices[0]);
        indices.push_back(face.mIndices[1]);
        indices.push_back(face.mIndices[2]);
    }

    meshes[index].init (vertices, indices);
  }

  void RenderableMesh::render ()
  {
    applyTransform ();
    glEnableVertexAttribArray (0);
    glEnableVertexAttribArray (1);
    glEnableVertexAttribArray (2);

    for (unsigned int i = 0 ; i < meshes.size() ; i++)
    {
      glBindBuffer (GL_ARRAY_BUFFER, meshes[i].vbo);
      glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), 0);
      glVertexAttribPointer (1, 2, GL_FLOAT, GL_FALSE, sizeof (Vertex), (const GLvoid*) 12);
      glVertexAttribPointer (2, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), (const GLvoid*) 20);

      glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, meshes[i].ibo);

//      const unsigned int MaterialIndex = meshes[i].MaterialIndex;
//
//      if (MaterialIndex < textures.size() && textures[MaterialIndex])
//      {
//        textures[MaterialIndex]->Bind (GL_TEXTURE0);
//      }

      glDrawElements (GL_TRIANGLES, meshes[i].num_indices, GL_UNSIGNED_INT, 0);
    }

    glDisableVertexAttribArray (0);
    glDisableVertexAttribArray (1);
    glDisableVertexAttribArray (2);
    unapplyTransform ();
  }
}


//  void URDFRenderer::createBoxVBO (const urdf::Vector3 &dim, GLuint &vbo, GLuint &ibo)
//  {
//    int num_vert = 20; 
//    static const GLfloat boxvertices[] = {-1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                           1.0f * dim.x,  1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x,  1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                          -1.0f * dim.x,  1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x,  1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x,  1.0f * dim.y, -1.0f * dim.z,
//                                           1.0f * dim.x,  1.0f * dim.y,  1.0f * dim.z,
//                                           1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x, -1.0f * dim.y, -1.0f * dim.z,
//                                          -1.0f * dim.x, -1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x,  1.0f * dim.y,  1.0f * dim.z,
//                                          -1.0f * dim.x,  1.0f * dim.y, -1.0f * dim.z };
//
//    glGenBuffers (1, &vbo);
//    glBindBuffer (GL_ARRAY_BUFFER, vbo);
//    glBufferData(GL_ARRAY_BUFFER, 20 * 3 * sizeof(GLfloat), boxvertices, GL_STATIC_DRAW);
//
//    ushort pindices[num_vert];
//    for (int i = 0; i < num_vert; i++)
//      pindices[i] = i;
//
//    glGenBuffers(1, &ibo);
//    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
//    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(ushort)*num_vert, pindices, GL_STATIC_DRAW);
//  }
//    {
//      GLuint vbo = it->second.first;
//      GLuint ibo = it->second.second;
//      glBindBuffer (GL_ARRAY_BUFFER, vbo);
//      glEnableVertexAttribArray (0);
//      glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(GLfloat), BUFFER_OFFSET(0));
//
//      glBindBuffer (GL_ELEMENT_ARRAY_BUFFER, ibo);
//      glDrawElements (GL_TRIANGLES, 3, GL_UNSIGNED_SHORT, BUFFER_OFFSET(0));
//      glDisableVertexAttribArray (0);
//    }
