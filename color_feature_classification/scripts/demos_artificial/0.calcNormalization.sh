#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_artificial

mkdir bin_normalization

files=`find $DATA/features_c/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/minmax_c.txt

files=`find $DATA/features_d/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/minmax_d.txt

files=`find $DATA/features_g/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/minmax_g.txt

files=`find $DATA/features_r/ -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calcNormalization $files bin_normalization/minmax_r.txt