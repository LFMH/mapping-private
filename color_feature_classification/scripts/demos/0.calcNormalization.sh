#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

mkdir -p bin_normalization

files=`find $DATA/test_features_c/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/max_c.txt

files=`find $DATA/test_features_d/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/max_d.txt

files=`find $DATA/test_features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/max_g.txt

files=`find $DATA/test_features_r/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/max_r.txt