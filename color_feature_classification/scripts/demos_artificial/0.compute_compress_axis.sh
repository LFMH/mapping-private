#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data/

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/minmax_c.txt"
norm_flag_d="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/minmax_d.txt"
norm_flag_g="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/minmax_g.txt"
norm_flag_r="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/minmax_r.txt"

# compute a subspace
files=`find features_c/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_c pca_result_c/compress_axis
files=`find features_d/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_d pca_result_d/compress_axis
files=`find features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_g pca_result_g/compress_axis
files=`find features_r/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_r pca_result_r/compress_axis

# files=`find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \)`
# rosrun color_feature_classification computeSubspace_with_rotate c $files -rotate 1 -subdiv 5 -offset 2 pca_result_c/compress_axis
# rosrun color_feature_classification computeSubspace_with_rotate d $files -rotate 1 -subdiv 5 -offset 2 pca_result_d/compress_axis
# rosrun color_feature_classification computeSubspace g $files -subdiv 5 -offset 2 pca_result_g/compress_axis
# rosrun color_feature_classification computeSubspace r $files -subdiv 5 -offset 2 pca_result_r/compress_axis
