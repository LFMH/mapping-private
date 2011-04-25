#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

# compute a subspace
files=`find $DATA/features_c/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_c $DATA/pca_result_c/compress_axis
files=`find $DATA/features_d/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_d $DATA/pca_result_d/compress_axis
files=`find $DATA/features_g/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_g $DATA/pca_result_g/compress_axis
files=`find $DATA/features_r/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_r $DATA/pca_result_r/compress_axis

# files=`find $DATA -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
# rosrun color_feature_classification computeSubspace_with_rotate c $files -rotate 1 -subdiv 10 -offset 5 pca_result_c/compress_axis
# rosrun color_feature_classification computeSubspace_with_rotate d $files -rotate 1 -subdiv 10 -offset 5 pca_result_d/compress_axis
# rosrun color_feature_classification computeSubspace g $files -subdiv 10 -offset 5 pca_result_g/compress_axis
# rosrun color_feature_classification computeSubspace r $files -subdiv 10 -offset 5 pca_result_r/compress_axis
