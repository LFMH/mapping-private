#!/bin/bash
# Example directory containing .pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric

# NOTE: comment-out the followings if you don't use normalization
norm_flag_g="-norm $DIR/bin_normalization/max_g.txt"

# compute a subspace
files=`find $DATA/training_features_g/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_g $DIR/pca_result_g/compress_axis
