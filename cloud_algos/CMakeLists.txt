cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

ROSBUILD_ADD_LIBRARY (cloud_algos
#  src/normal_estimation.cpp
#  src/planar_estimation.cpp
#  src/cylinder_fit.cpp
#  src/box_fit.cpp
#  src/gfpfh.cpp
  src/misc.cpp
  src/plugin_loader.cpp
  src/registration.cpp
  src/box_fit_algo.cpp
  src/mls_fit.cpp
#  src/radius_estimation.cpp
  src/pfh.cpp
  src/rotational_estimation.cpp
  src/region_growing.cpp
  src/register_plugins.cpp
  src/depth_image_triangulation.cpp
  src/svm_classification.cpp
  src/decision_manager.cpp
#  src/StringTokenizer.cpp
)

#rosbuild_add_executable (cloud_algos_test src/cloud_algos_test.cpp)
#target_link_libraries (cloud_algos_test cloud_algos)

#rosbuild_add_executable (normal_estimation_node src/normal_estimation.cpp)
#set_target_properties (normal_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#rosbuild_add_executable (planar_estimation_node src/planar_estimation.cpp)
#set_target_properties (planar_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (plugin_loader_node src/plugin_loader.cpp src/misc.cpp)
set_target_properties (plugin_loader_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (rotational_estimation_node src/rotational_estimation.cpp src/misc.cpp)
set_target_properties (rotational_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (registration_node src/registration.cpp src/misc.cpp)
set_target_properties (registration_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (mls_fit_node src/mls_fit.cpp src/misc.cpp)
set_target_properties (mls_fit_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (box_fit_node src/box_fit_algo.cpp src/misc.cpp)
set_target_properties (box_fit_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (pfh_node src/pfh.cpp src/misc.cpp)
set_target_properties (pfh_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (svm_classification_node src/svm_classification.cpp src/misc.cpp)
set_target_properties (svm_classification_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (depth_image_triangulation_node src/depth_image_triangulation.cpp src/misc.cpp)
set_target_properties (depth_image_triangulation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (radius_estimation_node src/radius_estimation.cpp src/misc.cpp)
set_target_properties (radius_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (decision_manager_node src/decision_manager.cpp)
set_target_properties (decision_manager_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (cylinder_fit_node src/cylinder_fit.cpp src/misc.cpp)
#set_target_properties (cylinder_fit_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#DEPRECATED
#rosbuild_add_executable (gfpfh_node src/gfpfh.cpp)
#set_target_properties (gfpfh_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)
