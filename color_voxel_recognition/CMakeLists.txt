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

set(USE_OPENCV 1)
set(USE_PCL 1)

if(USE_OPENCV)
add_definitions(-DUSE_OPENCV)
endif(USE_OPENCV)
if(USE_PCL)
add_definitions(-DUSE_PCL)
endif(USE_PCL)

#########
# include(${CMAKE_SOURCE_DIR}/FindOctave.cmake)
# if(NOT OCTAVE_FOUND)
#   message(FATAL_ERROR "Octave installation not found")
# endif(NOT OCTAVE_FOUND)
# include_directories(${OCTAVE_INCLUDE_DIRS})
# link_directories(${OCTAVE_LINK_DIRS})
#########

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(GLUT)
find_package(OpenGL)
set(GRAPHICSLIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY})
message(${GLUT_LIBRARY})

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/CCHLAC.cpp src/Param.cpp src/Voxel.cpp src/libPCA.cpp src/objFile.cpp src/ppmFile.cpp src/glView.cpp src/Search.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(calc_scene_CCHLAC test/calc_scene_CCHLAC.cpp)
rosbuild_add_executable(calc_scene_CCHLAC_withPCAsolve test/calc_scene_CCHLAC_withPCAsolve.cpp)
rosbuild_add_executable(calc_scene_autoThreshold test/calc_scene_autoThreshold.cpp)
#rosbuild_add_executable(getMesh_model test/getMesh_model.cpp)
#rosbuild_add_executable(getMesh_scene test/getMesh_scene.cpp)
rosbuild_add_executable(getVoxel_model test/getVoxel_model.cpp)
#rosbuild_add_executable(getVoxel_scene test/getVoxel_scene.cpp)
rosbuild_add_executable(getVoxel_scene_each test/getVoxel_scene_each.cpp)
#rosbuild_add_executable(getColorPoint_model test/getColorPoint_model.cpp)
#rosbuild_add_executable(getColorPoint_scene test/getColorPoint_scene.cpp)
#rosbuild_add_executable(mkMask test/mkMask.cpp)
rosbuild_add_executable(pca_models test/pca_models.cpp)
#rosbuild_add_executable(pca_scene test/pca_scene.cpp)
rosbuild_add_executable(pca_scene_from_each_file test/pca_scene_from_each_file.cpp)
#rosbuild_add_executable(search test/search.cpp)
#rosbuild_add_executable(show_result test/show_result.cpp)
rosbuild_add_executable(variance_show test/variance_show.cpp)
#rosbuild_add_executable(view_mesh test/view_mesh.cpp)
#rosbuild_add_executable(view_result test/view_result.cpp)
rosbuild_add_executable(view_voxel test/view_voxel.cpp)
rosbuild_add_executable(saveData test/saveData.cpp)
rosbuild_add_executable(detectObj test/detectObj.cpp)
#rosbuild_add_executable(detectObj_torso test/detectObj_torso.cpp)
#rosbuild_add_executable(detectObj_VOSCH_multi test/detectObj_VOSCH_multi.cpp)
#target_link_libraries(calc_scene_CCHLAC ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(calc_scene_CCHLAC_withPCAsolve ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(getMesh_model ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(getMesh_scene ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(getVoxel_model ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(getVoxel_scene ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(getVoxel_scene_each ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(getColorPoint_model ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(getColorPoint_scene ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(mkMask ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(pca_models ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(pca_scene ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(pca_scene_from_each_file ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(search ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(variance_show ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(view_mesh ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(view_result ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(view_voxel ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(saveData ${PROJECT_NAME} ${GRAPHICSLIBS})
target_link_libraries(detectObj ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(detectObj_torso ${PROJECT_NAME} ${GRAPHICSLIBS})
#target_link_libraries(detectObj_VOSCH_multi ${PROJECT_NAME} ${GRAPHICSLIBS})