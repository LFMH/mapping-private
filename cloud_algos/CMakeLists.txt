cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Debug) #RelWithDebInfo)
rosbuild_init ()
rosbuild_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

ROSBUILD_ADD_LIBRARY (cloud_algos
#  src/normal_estimation.cpp
#  src/planar_estimation.cpp
#  src/gfpfh.cpp
#  src/StringTokenizer.cpp
#  src/decision_manager.cpp
  src/misc.cpp
#  src/plugin_loader.cpp
  src/registration.cpp
  src/cylinder_fit_algo.cpp
  src/radius_estimation.cpp
  src/pfh.cpp
  src/rotational_estimation.cpp
  src/region_growing.cpp
  src/register_plugins.cpp
  src/svm_classification.cpp
  src/noise_removal.cpp
)

#rosbuild_add_executable (cloud_algos_test src/cloud_algos_test.cpp)
#target_link_libraries (cloud_algos_test cloud_algos)

#rosbuild_add_executable (normal_estimation_node src/normal_estimation.cpp)
#set_target_properties (normal_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#rosbuild_add_executable (planar_estimation_node src/planar_estimation.cpp)
#set_target_properties (planar_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#rosbuild_add_executable (plugin_loader_node src/plugin_loader.cpp src/misc.cpp)
#set_target_properties (plugin_loader_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (rotational_estimation_node src/rotational_estimation.cpp src/misc.cpp)
set_target_properties (rotational_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (registration_node src/registration.cpp src/misc.cpp)
set_target_properties (registration_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (cylinder_estimation_node src/cylinder_fit_algo.cpp)
set_target_properties (cylinder_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (pfh_node src/pfh.cpp src/misc.cpp)
set_target_properties (pfh_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (svm_classification_node src/svm_classification.cpp src/misc.cpp)
set_target_properties (svm_classification_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (radius_estimation_node src/radius_estimation.cpp src/misc.cpp)
set_target_properties (radius_estimation_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

#rosbuild_add_executable (decision_manager_node src/decision_manager.cpp)
#set_target_properties (decision_manager_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

rosbuild_add_executable (statistical_noise_removal_node src/noise_removal.cpp)
set_target_properties (statistical_noise_removal_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)

##include_directories(bla ../pcl_to_octree/include/)
#rosbuild_add_executable(bla src/bla.cpp
##                            ../pcl_to_octree/src/pcl_to_octree/octree/OcTreePCL.cpp
##                            ../pcl_to_octree/src/pcl_to_octree/octree/OcTreeNodePCL.cpp
#)
#get_target_property(prev_props bla COMPILE_FLAGS)
#set_target_properties (bla PROPERTIES COMPILE_FLAGS "${prev_props} -DCREATE_NODE")
##add_definitions(-DCREATE_NODE)
#target_link_libraries(bla octree_pcl)

#set_target_properties (global_rsd_node PROPERTIES COMPILE_FLAGS -DCREATE_NODE)