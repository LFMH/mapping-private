cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init()

rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(EXECUTABLE_LIBRARY_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable (table_memory src/table_memory.cpp)
rosbuild_add_executable (table_memory_grsd src/table_memory_grsd.cpp)

