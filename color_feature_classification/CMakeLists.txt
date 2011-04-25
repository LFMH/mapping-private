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
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable (computeFeature test/computeFeature.cpp)
rosbuild_add_executable (computeGRSD test/computeGRSD.cpp)
rosbuild_add_executable (computeFeatureForSVM test/computeFeatureForSVM.cpp)
#rosbuild_add_executable (computeSubspace test/computeSubspace.cpp)
#rosbuild_add_executable (computeSubspace_with_rotate test/computeSubspace_with_rotate.cpp)
rosbuild_add_executable (computeSubspace_from_file test/computeSubspace_from_file.cpp)
#rosbuild_add_executable (test_classify test/test_classify.cpp)
rosbuild_add_executable (test_classify_from_file test/test_classify_from_file.cpp)
rosbuild_add_executable (test_classify_from_file_GRSD test/test_classify_from_file_GRSD.cpp)
rosbuild_add_executable (show_variance test/show_variance.cpp)
rosbuild_add_executable (cluster2global test/cluster2global.cpp)
rosbuild_add_executable (calc_autoThreshold test/calc_autoThreshold.cpp)
rosbuild_add_executable (calcNormalization test/calcNormalization.cpp)
rosbuild_add_executable (normalizeHist test/normalizeHist.cpp)
#rosbuild_add_executable (projectFeatures test/projectFeatures.cpp)
rosbuild_add_executable (convertOld2NewPCD test/convertOld2NewPCD.cpp)
