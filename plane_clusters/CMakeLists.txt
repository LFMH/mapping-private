cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(plane_clusters)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_executable(query_service_node src/query_service.cpp)
rospack_add_executable(plane_clusters_node src/plane_clusters.cpp)

