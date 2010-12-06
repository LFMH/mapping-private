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

#common commands for building c++ executables and libraries
rosbuild_add_library(color_chlac src/ColorCHLAC.cpp src/ColorVoxel.cpp)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(test_color_chlac test/example_colorCHLAC.cpp src/ColorCHLAC.cpp src/ColorVoxel.cpp)
rosbuild_add_executable(test_color_chlac test/example_colorCHLAC.cpp)
target_link_libraries(test_color_chlac color_chlac)
rosbuild_add_executable(test_color_chlac_grid test/example_colorCHLAC_grid.cpp)
target_link_libraries(test_color_chlac_grid color_chlac)
#target_link_libraries(example ${PROJECT_NAME})
