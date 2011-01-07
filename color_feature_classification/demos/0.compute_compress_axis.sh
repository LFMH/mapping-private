#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data

# compute a subspace
files=`find features_c/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files pca_result_c/compress_axis
files=`find features_d/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files pca_result_d/compress_axis
files=`find features_g/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files pca_result_g/compress_axis
files=`find features_r/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files pca_result_r/compress_axis

# files=`find $DATA -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
# rosrun color_feature_classification computeSubspace_with_rotate c $files -rotate 1 -subdiv 10 -offset 5 pca_result_c/compress_axis
# rosrun color_feature_classification computeSubspace_with_rotate d $files -rotate 1 -subdiv 10 -offset 5 pca_result_d/compress_axis
# rosrun color_feature_classification computeSubspace g $files -subdiv 10 -offset 5 pca_result_g/compress_axis
# rosrun color_feature_classification computeSubspace r $files -subdiv 10 -offset 5 pca_result_r/compress_axis
