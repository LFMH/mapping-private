#include "realtime_perception/offscreen_rendering.h"

namespace realtime_perception
{
  OffscreenRenderer::OffscreenRenderer () : init_(false)
  {
    initGLWindow (640, 480);
    init_ = true;
  }

  bool OffscreenRenderer::isValid ()
  {
    return init_;
  }

  ////////////////////////////////////////////////////////////////////////////////
  bool OffscreenRenderer::initGL (int w, int h)
  {
    int attribList[] = {GLX_RENDER_TYPE,        GLX_RGBA_BIT,
                        GLX_MAX_PBUFFER_WIDTH,  w,
                        GLX_MAX_PBUFFER_HEIGHT, h,
                        GLX_RED_SIZE,           4,
                        GLX_GREEN_SIZE,         4,
                        GLX_BLUE_SIZE,          4,
                        GLX_DRAWABLE_TYPE,      GLX_PBUFFER_BIT,
                        GLX_DEPTH_SIZE,         24,
                        None};

    // Open X11 display
    Display *dpy = XOpenDisplay (0);

    // Get default screen
    int screen = XDefaultScreen (dpy);

    // Configure frame buffer
    int fbconfigCount;
    GLXFBConfig *fbconfig = glXChooseFBConfig (dpy,
                                               screen,
                                               attribList,
                                               &fbconfigCount);

    if (fbconfig == NULL || fbconfigCount <= 0)
    {
      fprintf(stderr, "P-Buffers not supported.\n");
      return false;
    }

    // Create P-Buffer
    int attrib_2[]={
        GLX_PBUFFER_WIDTH, w,
        GLX_PBUFFER_HEIGHT, h,
        GLX_NONE
      };
    
    pbuffer = glXCreatePbuffer (dpy, fbconfig[0], attrib_2);
    
    if (pbuffer == None)
    {
      fprintf (stderr, "Failed to create P-Buffer.\n");
      return false;
    }

    // Create graphics context
    cx = glXCreateNewContext (dpy,
                              fbconfig[0],
                              GLX_RGBA_TYPE,
                              NULL,
                              GL_TRUE);
    
    if (!cx) 
    {
      fprintf(stderr, "Failed to create graphics context.\n");
      return false;
    }

    // Activate graphics context
    if (!glXMakeContextCurrent(dpy, pbuffer, pbuffer, cx))
    {
      fprintf(stderr, "Failed to activate graphics context.\n");
      return false;
    }

    // If all the preceding commands execute OK, then all the OpenGL instructions
    // will be executed in the offscreen buffer
    return true;
  }

  void OffscreenRenderer::initGLWindow (int w, int h)
  {
    //char **argv; int argc = 1;
    //argv = (char**)malloc (1*sizeof(char*));
    //argv[0] = (char*)malloc (4*sizeof(char));
    //glutInit(&argc, argv);
    glutInitDisplayMode (GLUT_SINGLE);
    glutInitWindowSize (w, h);
    glutInitWindowPosition (100, 100);
    glutCreateWindow ("offscreen rendering visualization (ironic, i know)");
    
    glViewport(0, 0, (GLsizei)w, (GLsizei)h); // Set our viewport to the size of our window  
    glMatrixMode(GL_PROJECTION); // Switch to the projection matrix so that we can manipulate how our scene is viewed  
    glLoadIdentity(); // Reset the projection matrix to the identity matrix so that we don't get any artifacts (cleaning up)  
    gluPerspective(60, (GLfloat)w / (GLfloat)h, 1.0, 100.0); // Set the Field of view angle (in degrees), the aspect ratio of our window, and the new and far planes  
    glMatrixMode(GL_MODELVIEW); // Switch back to the model view matrix, so that we can start drawing shapes correctly  
  }
} // end namespace



