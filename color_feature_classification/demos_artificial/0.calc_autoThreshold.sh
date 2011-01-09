#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data

# compute a subspace
files=`find $DATA -type f -iname "obj_*.pcd" | sort -d`
rosrun color_feature_classification calc_autoThreshold $files color_threshold.txt
