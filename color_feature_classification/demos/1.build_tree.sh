#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

# Chi-Square distance metric
metric=7

# Build a KD-Tree
# VFH
files=`find $DATA -type f \( -iname "*_vfh.pcd" \)`
rosrun color_feature_classification build_tree $files -metric $metric -linear 0 kdtree_vfh.idx training_data_vfh.h5 training_data_vfh.list

# colorCHLAC
files=`find $DATA -type f \( -iname "*_colorCHLAC.pcd" \)`
rosrun color_feature_classification build_tree $files -metric $metric -linear 0 kdtree_colorCHLAC.idx training_data_colorCHLAC.h5 training_data_colorCHLAC.list
