cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# TODO: Put data of table scenes with the segmetned objects

# ------[ Data
#rosbuild_download_data (http://pr.willowgarage.com/data/pcl/office_chair_model.pcd data/office_chair_model.pcd d23282d322e0d45cadbce2e23bfc9e60)

# ------[ Segment
rosbuild_add_executable (hough_segmentation src/segmentation_by_voting.cpp)
#rosbuild_add_executable (process_clouds src/process_clouds.cpp)
#rosbuild_add_executable (rsd src/rsd.cpp)
target_link_libraries (hough_segmentation pcl_io)
