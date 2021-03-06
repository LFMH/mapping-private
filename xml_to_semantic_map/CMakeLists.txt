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
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

INCLUDE_DIRECTORIES (/usr/include/libxml++-2.6)
INCLUDE_DIRECTORIES (/usr/include/glibmm-2.4)
INCLUDE_DIRECTORIES (/usr/include/glib-2.0)
INCLUDE_DIRECTORIES (/usr/lib/glib-2.0/include)
INCLUDE_DIRECTORIES (/usr/lib/glibmm-2.4/include)
INCLUDE_DIRECTORIES (/usr/lib/libxml++-2.6/include)

rosbuild_add_executable(xml_to_semantic_map src/xml_to_semantic_map.cpp src/StringTokenizer.cc)
target_link_libraries(xml_to_semantic_map glibmm-2.4 xml++-2.6)

rosbuild_add_executable(semantic_map_to_marker src/semantic_map_to_marker.cpp)
