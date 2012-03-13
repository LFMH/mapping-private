#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_grsd/data/rgbd-sel-smooth-reduced-arranged-noball6

mkdir -p `rospack find color_feature_classification`/demos_grsd/bin_normalization

files=`find $DATA/testing_features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files `rospack find color_feature_classification`/demos_grsd/bin_normalization/max_g.txt

files=`find $DATA/testing_features_c/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files `rospack find color_feature_classification`/demos_grsd/bin_normalization/max_c.txt

files=`find $DATA/testing_features_r/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files `rospack find color_feature_classification`/demos_grsd/bin_normalization/max_r.txt
