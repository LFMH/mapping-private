#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

for i in `find $DATA -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
do
    echo "Processing $i"
    rosrun color_feature_classification computeFeature $i v
    rosrun color_feature_classification computeFeature $i c
done