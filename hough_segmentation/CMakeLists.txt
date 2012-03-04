cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(HOUGH_SEGMENTATION)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(CMAKE_BUILD_TYPE Debug)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(sequential_hough src/sequential_hough.cpp)
target_link_libraries(sequential_hough ${PCL_LIBRARIES})

#add_executable(hough_classification src/hough_classification.cpp)
#target_link_libraries(hough_classification ${PCL_LIBRARIES})

add_executable(rot_trans_scale src/rot_trans_scale.cpp)
target_link_libraries(rot_trans_scale ${PCL_LIBRARIES})

#add_executable (cloud_operations src/cloud_operations.cpp)
#target_link_libraries(cloud_operations ${PCL_LIBRARIES})



#
## Old Version Which Uses Perception PCL 0.9.9 #
#
#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#
## Set the build type.  Options are:
##  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
##  Debug          : w/ debug symbols, w/o optimization
##  Release        : w/o debug symbols, w/ optimization
##  RelWithDebInfo : w/ debug symbols, w/ optimization
##  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
##set(ROS_BUILD_TYPE RelWithDebInfo)
#
#rosbuild_init()
#
##set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#
##uncomment if you have defined messages
##rosbuild_genmsg()
##uncomment if you have defined services
##rosbuild_gensrv()
#
## TODO: Put data of table scenes with the segmetned objects
#
## ------[ Data
##rosbuild_download_data (http://pr.willowgarage.com/data/pcl/office_chair_model.pcd data/office_chair_model.pcd d23282d322e0d45cadbce2e23bfc9e60)
#
## ------[ Segment
#rosbuild_add_executable (rsd src/rsd.cpp)
#rosbuild_add_executable (process_clouds src/process_clouds.cpp)
#rosbuild_add_executable (hough_segmentation src/segmentation_by_voting.cpp)
#rosbuild_add_executable (hough_segmentation_with_lines src/using_lines_for_segmentation_by_voting.cpp)
#rosbuild_add_executable (hough_segmentation_with_circles src/using_circles_for_segmentation_by_voting.cpp)
#target_link_libraries (hough_segmentation pcl_io)
#
