#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos/data

# compute a subspace
files=`find $DATA/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calc_autoThreshold $files color_threshold.txt
