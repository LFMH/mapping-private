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

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
# disabling this line makes the compiler use the system-wide install of pcl.
# this will conflict with the ros-wide pcl that gets pulled in through the manifest.xml
#find_package(PCL REQUIRED) 

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS})

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
rosbuild_add_executable(urdf_filter src/urdf_filter.cpp)
target_link_libraries(urdf_filter pcl_io pcl_cuda_io pcl_visualization pcl_common pcl_kdtree pcl_range_image)

rosbuild_add_executable(realtime_segmentation src/realtime_segmentation.cpp)
target_link_libraries (realtime_segmentation pcl_io pcl_cuda_io pcl_cuda_features pcl_cuda_segmentation pcl_cuda_sample_consensus pcl_visualization pcl_common pcl_kdtree pcl_range_image ${OpenCV_LIBS})
