#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data
n=0

# compute a subspace
for i in `find $DATA -type d -name "*"`
do
    echo $i
    num=$(printf "%03d" $n)
# VFH
    files=`find $i -type f \( -iname "*cluster*_vfh.pcd" \) | sort -R`
    rosrun color_feature_classification computeSubspace $files pca_result_vfh/$num

# colorCHLAC
    files=`find $i -type f \( -iname "*cluster*_colorCHLAC.pcd" \) | sort -R`
    rosrun color_feature_classification computeSubspace $files pca_result_colorCHLAC/$num

    n=`expr $n + 1`
done