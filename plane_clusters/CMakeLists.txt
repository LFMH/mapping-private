cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rospack(plane_clusters)
set(ROS_BUILD_TYPE Debug)
rosbuild_init()

# Must have <SWI-Prolog.h>
INCLUDE(CheckIncludeFiles)
CHECK_INCLUDE_FILES ("/usr/lib/swi-prolog/include/SWI-Prolog.h" HAVE_SWI_PROLOG_H)
if(HAVE_SWI_PROLOG_H)
  message(STATUS "<SWI-Prolog.h> found")
  INCLUDE_DIRECTORIES("/usr/lib/swi-prolog/include/")
else(HAVE_SWI_PROLOG_H)
  message(FATAL_ERROR "Can't proceed without <SWI-Prolog.h>")
endif(HAVE_SWI_PROLOG_H)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(plane_clusters_node src/plane_clusters.cpp)
#target_link_libraries(plane_clusters_node utilities)
rosbuild_add_executable(query_service_node src/query_service.cpp)
rosbuild_add_executable(detect_boxes_node src/detect_boxes.cpp)
rosbuild_add_executable(detect_planes_node src/detect_planes.cpp)

rosbuild_add_library(ros_plane_query src/query_service_prolog.cpp)
