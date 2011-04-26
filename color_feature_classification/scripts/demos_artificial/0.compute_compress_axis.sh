#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_artificial

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_c.txt"
norm_flag_d="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_d.txt"
norm_flag_g="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_g.txt"
norm_flag_r="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_r.txt"

mkdir -p $DATA/pca_result_c
mkdir -p $DATA/pca_result_d
mkdir -p $DATA/pca_result_g
mkdir -p $DATA/pca_result_r

# compute a subspace
files=`find $DATA/features_c/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_c $DATA/pca_result_c/compress_axis
files=`find $DATA/features_d/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_d $DATA/pca_result_d/compress_axis
files=`find $DATA/features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_g $DATA/pca_result_g/compress_axis
files=`find $DATA/features_r/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_r $DATA/pca_result_r/compress_axis