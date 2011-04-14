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
rosbuild_add_library(${PROJECT_NAME} src/Param.cpp src/libPCA.cpp src/Search.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(calc_scene_autoThreshold test/calc_scene_autoThreshold.cpp)
rosbuild_add_executable(extractC3HLAC_scene test/extractC3HLAC_scene.cpp)
rosbuild_add_executable(extractC3HLAC_models test/extractC3HLAC_models.cpp)
rosbuild_add_executable(pca_scene test/pca_scene.cpp)
rosbuild_add_executable(pca_models test/pca_models.cpp)
rosbuild_add_executable(variance_show test/variance_show.cpp)
rosbuild_add_executable(saveData test/saveData.cpp)
rosbuild_add_executable(detectObj test/detectObj.cpp)
##### rosbuild_add_executable(getVoxel_model test/getVoxel_model.cpp)
##### rosbuild_add_executable(getVoxel_scene test/getVoxel_scene.cpp)
target_link_libraries(calc_scene_autoThreshold ${PROJECT_NAME})
target_link_libraries(extractC3HLAC_scene ${PROJECT_NAME})
target_link_libraries(extractC3HLAC_models ${PROJECT_NAME})
target_link_libraries(pca_scene ${PROJECT_NAME})
target_link_libraries(pca_models ${PROJECT_NAME})
target_link_libraries(variance_show ${PROJECT_NAME})
target_link_libraries(detectObj ${PROJECT_NAME})
##### target_link_libraries(getVoxel_model ${PROJECT_NAME})
##### target_link_libraries(getVoxel_scene ${PROJECT_NAME})
