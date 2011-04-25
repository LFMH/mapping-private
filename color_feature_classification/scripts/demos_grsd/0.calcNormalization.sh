#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric

mkdir -p `rospack find color_feature_classification`/demos_grsd/bin_normalization

files=`find $DATA/test_training_features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files `rospack find color_feature_classification`/demos_grsd/bin_normalization/max_g.txt
