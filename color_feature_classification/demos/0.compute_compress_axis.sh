#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

# compute a subspace
# VFH
# files=`find $DATA -type f \( -iname "*_vfh.pcd" \)`
# rosrun color_feature_classification computeSubspace $files pca_result_vfh/compress_axis

# colorCHLAC
files=`find $DATA -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
rosrun color_feature_classification computeSubspace_with_rotate c $files -rotate 1 pca_result_c/compress_axis
#files=`find $DATA -type f \( -iname "*_colorCHLAC.pcd" \)`
#rosrun color_feature_classification computeSubspace $files pca_result_colorCHLAC/compress_axis
