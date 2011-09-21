#include <stdlib.h>
//#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
/* Ensure we are using opengl's core profile only */
#define GL3_PROTOTYPES 1
#include <GL3/gl3.h>
//#include <GL/gl.h>
#include <GL/freeglut.h>
#define GL_GLEXT_PROTOTYPES

#include <GL/glext.h>
#include <stdio.h>
#include <math.h>
 
#define PROGRAM_NAME "Tutorial2"

#include <errno.h>

std::string loadshader (std::string shader_file_name)
{
  std::ifstream ifs(shader_file_name.c_str());
  std::string str((std::istreambuf_iterator<char>(ifs)),
                  std::istreambuf_iterator<char>());
  return str;
}

char* textFileRead (const char *fn)
{
  FILE* fp;
  char* content = NULL;

  int count=0;

  if (fn != NULL)
  {
    fp = fopen(fn,"rt");
    if (fp != NULL)
    {
      fseek(fp, 0, SEEK_END);
      count = ftell(fp);
      rewind(fp);

      if (count > 0)
      {
        content = (char*) malloc (sizeof(char) * (count + 1));
        count = fread (content, sizeof(char), count, fp);
        content[count] = '\0';
      }
      else
        std::cerr << "count is zero" << std::endl;
      fclose(fp);
    }
    else
      std::cerr << "could not open file " << errno << std::endl;
  }
  else
    std::cerr << "Idiot" << std::endl;
  return content;
}

struct ShaderLoader
{
  ShaderLoader (std::string vertex, std::string fragment)
  {
    /* These strings will receive the contents of our shader source code files */
    std::string vertexsource, fragmentsource;

    /* Read our shaders into the appropriate buffers */
    vertexsource = readfile (vertex);
    fragmentsource = readfile (fragment);

    const char* vertexsource2 = textFileRead (vertex.c_str());
    const char* fragmentsource2 = textFileRead (fragment.c_str());

    std::cerr << vertexsource2 << std::endl << std::endl;
    std::cerr << fragmentsource2 << std::endl << std::endl;

    if (!compileshader (vertexsource2, GL_VERTEX_SHADER, vertexshader))
      return;
    if (!compileshader (fragmentsource2, GL_FRAGMENT_SHADER, fragmentshader))
      return;

    std::cout << "Compiled Vertex and Fragment Shaders." << std::endl;
    std::cerr << "ASD V " << vertexshader << std::endl;
    std::cerr << "ASD F " << fragmentshader << std::endl;


    /* If we reached this point it means the vertex and fragment shaders compiled and are syntax error free. */
    /* We must link them together to make a GL shader program */
    /* GL shader programs are monolithic. It is a single piece made of 1 vertex shader and 1 fragment shader. */
    /* Assign our program handle a "name" */
    shaderprogram = glCreateProgram();

    /* Attach our shaders to our program */
    glAttachShader(shaderprogram, vertexshader);
    glAttachShader(shaderprogram, fragmentshader);

    /* Bind attribute index 0 (coordinates) to in_Position and attribute index 1 (color) to in_Color */
    /* Attribute locations must be setup before calling glLinkProgram. */
    glBindAttribLocation(shaderprogram, 0, "in_Position");
    glBindAttribLocation(shaderprogram, 1, "in_Color");

    /* Link our program */
    /* At this stage, the vertex and fragment programs are inspected, optimized and a binary code is generated for the shader. */
    /* The binary code is uploaded to the GPU, if there is no error. */
    glLinkProgram(shaderprogram);

    /* Again, we must check and make sure that it linked. If it fails, it would mean either there is a mismatch between the vertex */
    /* and fragment shaders. It might be that you have surpassed your GPU's abilities. Perhaps too many ALU operations or */
    /* too many texel fetch instructions or too many interpolators or dynamic loops. */

    GLint IsLinked = GL_TRUE;
    glGetProgramiv(shaderprogram, GL_LINK_STATUS, &IsLinked);
    std::cerr << "asd " << 
    glGetError ()
    << std::endl;
    GLint other = 0;
    glGetProgramiv(shaderprogram, GL_ATTACHED_SHADERS, &other);
    std::cerr << "asd " << other << std::endl;
    glGetProgramiv(shaderprogram, GL_ACTIVE_ATTRIBUTES, &other);
    std::cerr << "asd " << other << std::endl;
    if (!IsLinked)
    {
      int maxLength = 0;
      /* Noticed that glGetProgramiv is used to get the length for a shader program, not glGetShaderiv. */
      glGetProgramiv(shaderprogram, GL_INFO_LOG_LENGTH, &maxLength);

      /* The maxLength includes the NULL character */
      char *shaderProgramInfoLog = (char *)malloc(maxLength);

      /* Notice that glGetProgramInfoLog, not glGetShaderInfoLog. */
      glGetProgramInfoLog(shaderprogram, maxLength, &maxLength, shaderProgramInfoLog);

      /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
      if (maxLength == 0)
        std::cerr << "Unable to link shader (no program info log available)." << std::endl;
      else
        std::cerr << "Unable to link shader [" << maxLength << "]: " << shaderProgramInfoLog << std::endl;
      /* In this simple program, we'll just leave */
      free (shaderProgramInfoLog);
      return;
    }
  }

  ~ShaderLoader ()
  {
    /* Cleanup all the things we bound and allocated */
    glUseProgram(0);
    glDetachShader(shaderprogram, vertexshader);
    glDetachShader(shaderprogram, fragmentshader);
    glDeleteProgram(shaderprogram);
    glDeleteShader(vertexshader);
    glDeleteShader(fragmentshader);
  }

  bool compileshader (const char* shadersource, GLuint shadertype, GLuint &shaderhandle)
  {
    /* Create an empty vertex shader handle */
    shaderhandle = glCreateShader(shadertype);
    std::cerr << "ASD " << shaderhandle << std::endl;

    /* Send the vertex shader source code to GL */
    /* Note that the source code is NULL character terminated. */
    /* GL will automatically detect that therefore the length info can be 0 in this case (the last parameter) */
    const GLchar* shader = shadersource;
    glShaderSource(shaderhandle, 1, &shader, 0);

    /* Compile the vertex shader */
    glCompileShader(shaderhandle);

    int IsCompiled = GL_TRUE;
    glGetShaderiv(shaderhandle, GL_COMPILE_STATUS, &IsCompiled);
    if (IsCompiled == GL_FALSE)
    {
      int maxLength;
      glGetShaderiv(shaderhandle, GL_INFO_LOG_LENGTH, &maxLength);

      /* The maxLength includes the NULL character */

      char* infoLog = (char *)malloc(maxLength);

      glGetShaderInfoLog(shaderhandle, maxLength, &maxLength, infoLog);

      /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
      std::cerr << "Unable to compile shader: " << infoLog << std::endl;
      /* In this simple program, we'll just leave */
      free(infoLog);
      return false;
    }
    return true;
  }
  bool compileshader (std::string shadersource, GLuint shadertype, GLuint &shaderhandle)
  {
    /* Create an empty vertex shader handle */
    shaderhandle = glCreateShader(shadertype);
    std::cerr << "ASD " << shaderhandle << std::endl;

    /* Send the vertex shader source code to GL */
    /* Note that the source code is NULL character terminated. */
    /* GL will automatically detect that therefore the length info can be 0 in this case (the last parameter) */
    const GLchar* shader = shadersource.c_str();
    glShaderSource(shaderhandle, 1, &shader, 0);

    /* Compile the vertex shader */
    glCompileShader(shaderhandle);

    int IsCompiled;
    glGetShaderiv(shaderhandle, GL_COMPILE_STATUS, &IsCompiled);
    if (IsCompiled == GL_FALSE)
    {
      int maxLength;
      glGetShaderiv(shaderhandle, GL_INFO_LOG_LENGTH, &maxLength);

      /* The maxLength includes the NULL character */

      char* infoLog = (char *)malloc(maxLength);

      glGetShaderInfoLog(shaderhandle, maxLength, &maxLength, infoLog);

      /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
      std::cerr << "Unable to compile shader: " << infoLog << std::endl;
      /* In this simple program, we'll just leave */
      free(infoLog);
      return false;
    }
    return true;
  }

  std::string readfile (std::string shader_file_name)
  {
    std::ifstream ifs(shader_file_name.c_str());
    std::string str((std::istreambuf_iterator<char>(ifs)),
                    std::istreambuf_iterator<char>());
    return str;
  }

  GLuint shaderprogram;
  GLuint vertexshader, fragmentshader;
};



void prepare_shaders (GLuint &shaderprogram, GLuint &vertexshader, GLuint &fragmentshader, GLuint &vao, GLuint (&vbo)[2])
{
  int i; /* Simple iterator */
  int IsCompiled_VS, IsCompiled_FS;
  int IsLinked;
  int maxLength;
  char *vertexInfoLog;
  char *fragmentInfoLog;
  char *shaderProgramInfoLog;

  /* We're going to create a simple diamond made from lines */
  const GLfloat diamond[4][2] = {
  {  0.0,  1.0  }, /* Top point */
  {  1.0,  0.0  }, /* Right point */
  {  0.0, -1.0  }, /* Bottom point */
  { -1.0,  0.0  } }; /* Left point */

  const GLfloat colors[4][3] = {
  {  1.0,  0.0,  0.0  }, /* Red */
  {  0.0,  1.0,  0.0  }, /* Green */
  {  0.0,  0.0,  1.0  }, /* Blue */
  {  1.0,  1.0,  1.0  } }; /* White */

  /* These pointers will receive the contents of our shader source code files */
  std::string vertexsource, fragmentsource;

  /* Allocate and assign a Vertex Array Object to our handle */
  glGenVertexArrays(1, &vao);

  /* Bind our Vertex Array Object as the current used object */
  glBindVertexArray(vao);

  /* Allocate and assign two Vertex Buffer Objects to our handle */
  glGenBuffers(2, vbo);

  /* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
  glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);

  /* Copy the vertex data from diamond to our buffer */
  /* 8 * sizeof(GLfloat) is the size of the diamond array, since it contains 8 GLfloat values */
  glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(GLfloat), diamond, GL_STATIC_DRAW);

  /* Specify that our coordinate data is going into attribute index 0, and contains two floats per vertex */
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

  /* Enable attribute index 0 as being used */
  glEnableVertexAttribArray(0);

  /* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
  glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

  /* Copy the color data from colors to our buffer */
  /* 12 * sizeof(GLfloat) is the size of the colors array, since it contains 12 GLfloat values */
  glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(GLfloat), colors, GL_STATIC_DRAW);

  /* Specify that our color data is going into attribute index 1, and contains three floats per vertex */
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

  /* Enable attribute index 1 as being used */
  glEnableVertexAttribArray(1);

  /* Read our shaders into the appropriate buffers */
  vertexsource = loadshader ("../include/shaders/test1.vert");
  fragmentsource = loadshader ("../include/shaders/test1.frag");

  /* Create an empty vertex shader handle */
  vertexshader = glCreateShader(GL_VERTEX_SHADER);

  /* Send the vertex shader source code to GL */
  /* Note that the source code is NULL character terminated. */
  /* GL will automatically detect that therefore the length info can be 0 in this case (the last parameter) */
  const GLchar* shader = vertexsource.c_str();
  glShaderSource(vertexshader, 1, &shader, 0);

  /* Compile the vertex shader */
  glCompileShader(vertexshader);

  glGetShaderiv(vertexshader, GL_COMPILE_STATUS, &IsCompiled_VS);
  if (! IsCompiled_VS)
  {
    int maxLength;
    glGetShaderiv(vertexshader, GL_INFO_LOG_LENGTH, &maxLength);

    /* The maxLength includes the NULL character */
    vertexInfoLog = (char *)malloc(maxLength);

    glGetShaderInfoLog(vertexshader, maxLength, &maxLength, vertexInfoLog);

    /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
    /* In this simple program, we'll just leave */
    free(vertexInfoLog);
    return;
  }

  /* Create an empty fragment shader handle */
  fragmentshader = glCreateShader(GL_FRAGMENT_SHADER);

  /* Send the fragment shader source code to GL */
  /* Note that the source code is NULL character terminated. */
  /* GL will automatically detect that therefore the length info can be 0 in this case (the last parameter) */
  shader = fragmentsource.c_str();
  glShaderSource(fragmentshader, 1, &shader, 0);

  /* Compile the fragment shader */
  glCompileShader(fragmentshader);

  glGetShaderiv(fragmentshader, GL_COMPILE_STATUS, &IsCompiled_FS);
  if (! IsCompiled_FS)
  {
     glGetShaderiv(fragmentshader, GL_INFO_LOG_LENGTH, &maxLength);

     /* The maxLength includes the NULL character */
     fragmentInfoLog = (char *)malloc(maxLength);

     glGetShaderInfoLog(fragmentshader, maxLength, &maxLength, fragmentInfoLog);

     /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
     /* In this simple program, we'll just leave */
     free(fragmentInfoLog);
     return;
  }

  /* If we reached this point it means the vertex and fragment shaders compiled and are syntax error free. */
  /* We must link them together to make a GL shader program */
  /* GL shader programs are monolithic. It is a single piece made of 1 vertex shader and 1 fragment shader. */
  /* Assign our program handle a "name" */
  shaderprogram = glCreateProgram();

  /* Attach our shaders to our program */
  glAttachShader(shaderprogram, vertexshader);
  glAttachShader(shaderprogram, fragmentshader);

  /* Bind attribute index 0 (coordinates) to in_Position and attribute index 1 (color) to in_Color */
  /* Attribute locations must be setup before calling glLinkProgram. */
  glBindAttribLocation(shaderprogram, 0, "in_Position");
  glBindAttribLocation(shaderprogram, 1, "in_Color");

  /* Link our program */
  /* At this stage, the vertex and fragment programs are inspected, optimized and a binary code is generated for the shader. */
  /* The binary code is uploaded to the GPU, if there is no error. */
  glLinkProgram(shaderprogram);

  /* Again, we must check and make sure that it linked. If it fails, it would mean either there is a mismatch between the vertex */
  /* and fragment shaders. It might be that you have surpassed your GPU's abilities. Perhaps too many ALU operations or */
  /* too many texel fetch instructions or too many interpolators or dynamic loops. */

  glGetProgramiv(shaderprogram, GL_LINK_STATUS, (int *)&IsLinked);
  if (! IsLinked)
  {
     /* Noticed that glGetProgramiv is used to get the length for a shader program, not glGetShaderiv. */
     glGetProgramiv(shaderprogram, GL_INFO_LOG_LENGTH, &maxLength);

     /* The maxLength includes the NULL character */
     shaderProgramInfoLog = (char *)malloc(maxLength);

     /* Notice that glGetProgramInfoLog, not glGetShaderInfoLog. */
     glGetProgramInfoLog(shaderprogram, maxLength, &maxLength, shaderProgramInfoLog);

     /* Handle the error in an appropriate way such as displaying a message or writing to a log file. */
     /* In this simple program, we'll just leave */
     free(shaderProgramInfoLog);
     return;
  }

  /* Load the shader into the rendering pipeline */
  glUseProgram(shaderprogram);
}


void display ()
{
  glClearColor(0.0, 0.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  glColor4f(1,1,1,1);
  glTranslatef (0,0,-1);
  glutSolidCube (1.0);
  glutSwapBuffers ();
  glutPostRedisplay ();
}

void glInit (int w, int h)
{
  // set up our OpenGL state
  glEnable(GL_TEXTURE_3D);  // enable 3d texturing
  glEnable(GL_DEPTH_TEST);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // our texture colors will replace the untextured colors
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glColor4d(0.0, 0.0, 0.0, 1.0);
  glPointSize(3.0);
  
  glShadeModel(GL_FLAT);
  
  glViewport (0, 0, (GLsizei) 640, (GLsizei) 480);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90, 4.0/3.0, 0.1, 10.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
 
/* Our program's entry point */
int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize (640, 480);
  glutInitWindowPosition(80, 80);
  glutCreateWindow ("shader_test");
  
  glutDisplayFunc (display);
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
 // // VERY IMPORTANT:
 // // this line loads the address of the glTexImage3D function into the function pointer of the same name.
 // // glTexImage3D is not implemented in the standard GL libraries and must be loaded dynamically at run time,
 // // the environment the program is being run in MAY OR MAY NOT support it, if not we'll get back a NULL pointer.
 // // this is necessary to use any OpenGL function declared in the glext.h header file
 // // the Pointer to FunctioN ... PROC types are declared in the same header file with a type appropriate to the function name
 // glTexImage3D = (PFNGLTEXIMAGE3DPROC) wglGetProcAddress("glTexImage3D");
 // if (glTexImage3D == NULL) {
 //   printf("Error in line %d: Couldn't load glTexImage3D function. Aborting.\n", __LINE__);
 //   return -1;
 // }
  
  glInit (640, 480);
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

GLuint vao, vbo[2]; /* Create handles for our Vertex Array Object and two Vertex Buffer Objects */

/* We're going to create a simple diamond made from lines */
const GLfloat diamond[4][2] = {
{  0.0,  1.0  }, /* Top point */
{  1.0,  0.0  }, /* Right point */
{  0.0, -1.0  }, /* Bottom point */
{ -1.0,  0.0  } }; /* Left point */

const GLfloat colors[4][3] = {
{  1.0,  0.0,  0.0  }, /* Red */
{  0.0,  1.0,  0.0  }, /* Green */
{  0.0,  0.0,  1.0  }, /* Blue */
{  1.0,  1.0,  1.0  } }; /* White */

/* Allocate and assign a Vertex Array Object to our handle */
glGenVertexArrays(1, &vao);

/* Bind our Vertex Array Object as the current used object */
glBindVertexArray(vao);

/* Allocate and assign two Vertex Buffer Objects to our handle */
glGenBuffers(2, vbo);

/* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);

/* Copy the vertex data from diamond to our buffer */
/* 8 * sizeof(GLfloat) is the size of the diamond array, since it contains 8 GLfloat values */
glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(GLfloat), diamond, GL_STATIC_DRAW);

/* Specify that our coordinate data is going into attribute index 0, and contains two floats per vertex */
glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

/* Enable attribute index 0 as being used */
glEnableVertexAttribArray(0);

/* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

/* Copy the color data from colors to our buffer */
/* 12 * sizeof(GLfloat) is the size of the colors array, since it contains 12 GLfloat values */
glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(GLfloat), colors, GL_STATIC_DRAW);

/* Specify that our color data is going into attribute index 1, and contains three floats per vertex */
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

/* Enable attribute index 1 as being used */
glEnableVertexAttribArray(1);

  /* Compile, Link and Load shaders. */
  ShaderLoader sl ("include/shaders/test1.vert", "include/shaders/test1.frag");
  ShaderWrapper sl ("include/")
  glutMainLoop ();

  return 0;
}

//////////////////////////////////////////////////////////
// lalala
//  int windownumber;

//  glutPostRedisplay();
//  glutSwapBuffers();


