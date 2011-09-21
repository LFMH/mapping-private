#ifndef REALTIME_PERCEPTION_OFFSCREEN_RENDERING_H_
#define REALTIME_PERCEPTION_OFFSCREEN_RENDERING_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/gl.h>
#include <GL/freeglut.h>
#include <GL/glx.h>
 
#define GLX_CONTEXT_MAJOR_VERSION_ARB       0x2091
#define GLX_CONTEXT_MINOR_VERSION_ARB       0x2092

namespace realtime_perception
{
  class OffscreenRenderer
  {
    public:
      GLXContext cx;
      GLXPbuffer pbuffer;
      bool init_;

      OffscreenRenderer ();

      bool isValid ();

      bool initGL (int w, int h);

      static void initGLWindow (int w, int h);
  };

} // end namespace

#endif
