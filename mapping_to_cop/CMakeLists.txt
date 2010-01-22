PROJECT(mapping_to_cop)

cmake_minimum_required(VERSION 2.6)

INCLUDE("${CMAKE_MODULE_PATH}/MakeDocu.cmake" OPTIONAL)


include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

rosbuild_gensrv()
rosbuild_genmsg()

rosbuild_add_executable(mapping_to_cop source/main.cpp)


