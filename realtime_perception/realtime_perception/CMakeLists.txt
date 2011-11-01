cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(CUDA REQUIRED)
cmake_policy(SET CMP0012 NEW)
find_package(OpenCV REQUIRED)
find_package(OpenGL )
FIND_LIBRARY( freeglut_LIBRARY glut /usr/lib)

# disabling this line makes the compiler use the system-wide install of pcl.
# this will conflict with the ros-wide pcl that gets pulled in through the manifest.xml
#find_package(PCL REQUIRED) 

#include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIR})
FIND_PATH( freeglut_INCLUDE_DIR GL/freeglut.h
  /usr/include/GL
  /usr/openwin/share/include
  /usr/openwin/include
  /opt/graphics/OpenGL/include
  /opt/graphics/OpenGL/contrib/libglut
  )
include_directories(${freeglut_INCLUDE_DIR})

include(CudaComputeTargetFlags.cmake)
APPEND_TARGET_ARCH_FLAGS()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

############################################## RIPPED FROM rosbuild/public.cmake
# A wrapper around add_executable(), using info from the rospack
# invocation to set up compiling and linking.
macro(rosbuild_cuda_add_executable exe)
  cuda_add_executable(${ARGV})

  # Add explicit dependency of each file on our manifest.xml and those of
  # our dependencies.
  # The SOURCES property seems to be available only since 2.6.  Yar.
  #get_target_property(_srclist ${exe} SOURCES) 
  set(_srclist ${ARGN})
  foreach(_src ${_srclist}) 
    # Handle the case where the second argument is EXCLUDE_FROM_ALL, not a
    # source file.  Only have to do this because we can't get the SOURCES
    # property.
    if(NOT _src STREQUAL EXCLUDE_FROM_ALL)
      set(_file_name _file_name-NOTFOUND)
      find_file(_file_name ${_src} ${CMAKE_CURRENT_SOURCE_DIR} /)
      if(NOT _file_name)
        message("[rosbuild] Couldn't find source file ${_src}; assuming that it is in ${CMAKE_CURRENT_SOURCE_DIR} and will be generated later")
        set(_file_name ${CMAKE_CURRENT_SOURCE_DIR}/${_src})
      endif(NOT _file_name)
      add_file_dependencies(${_file_name} ${ROS_MANIFEST_LIST}) 
    endif(NOT _src STREQUAL EXCLUDE_FROM_ALL)
  endforeach(_src)

  rosbuild_add_compile_flags(${exe} ${${PROJECT_NAME}_CFLAGS_OTHER})
  rosbuild_add_link_flags(${exe} ${${PROJECT_NAME}_LDFLAGS_OTHER})

  if(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    # This will probably only work on Linux.  The LINK_SEARCH_END_STATIC
    # property should be sufficient, but it doesn't appear to work
    # properly.
    rosbuild_add_link_flags(${exe} -static-libgcc -Wl,-Bstatic)
  endif(ROS_BUILD_STATIC_EXES AND ${CMAKE_SYSTEM_NAME} STREQUAL "Linux")

  target_link_libraries(${exe} ${${PROJECT_NAME}_LIBRARIES})

  # Add ROS-wide compile and link flags (usually things like -Wall).  These
  # are set in rosconfig.cmake.
  rosbuild_add_compile_flags(${exe} ${ROS_COMPILE_FLAGS})
  rosbuild_add_link_flags(${exe} ${ROS_LINK_FLAGS})

  # Make sure to do any prebuild work (e.g., msg/srv generation) before
  # building this target.
  add_dependencies(${exe} rosbuild_precompile)

  # If we're linking boost statically, we have to force allow multiple definitions because
  # rospack does not remove duplicates
  if ("$ENV{ROS_BOOST_LINK}" STREQUAL "static")
    rosbuild_add_link_flags(${exe} "-Wl,--allow-multiple-definition")
  endif("$ENV{ROS_BOOST_LINK}" STREQUAL "static")

endmacro(rosbuild_cuda_add_executable)
############################################## RIPPED FROM rosbuild/public.cmake

rosbuild_add_library (FBO src/FrameBufferObject.cpp)
target_link_libraries (FBO GLEW)
rosbuild_add_library (shaderwrapper src/shader_wrapper.cpp)

rosbuild_cuda_add_executable (normals_and_smoothing
  src/normals_and_smoothing.cpp 
  src/urdf_renderer.cpp 
  src/renderable.cpp 
  src/urdf_filtering.cu
  #src/offscreen_rendering.cpp
  )
target_link_libraries (normals_and_smoothing
  ${OPENGL_LIBRARIES} ${freeglut_LIBRARY} 
  pcl_io pcl_cuda_io pcl_common pcl_cuda_features pcl_cuda_segmentation pcl_cuda_sample_consensus 
  ${OpenCV_LIBS} 
  FBO shaderwrapper)
rosbuild_cuda_add_executable (offscreen_test 
  src/offscreen_test.cpp 
  src/urdf_renderer.cpp 
  src/renderable.cpp 
  src/urdf_filtering.cu
  #src/offscreen_rendering.cpp
  )
target_link_libraries (offscreen_test 
  ${OPENGL_LIBRARIES} ${freeglut_LIBRARY} 
  pcl_io pcl_cuda_io pcl_common pcl_cuda_features pcl_cuda_segmentation pcl_cuda_sample_consensus 
  ${OpenCV_LIBS} 
  FBO shaderwrapper)

