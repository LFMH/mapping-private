#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_grsd

# compute a subspace
files=`find $DATA/data/rgbd-sel-smooth-reduced-arranged-noball6/training/* -type f -iname "*.pcd" | sort -d`
rosrun color_feature_classification calc_autoThreshold $files $DATA $DATA/color_threshold.txt
