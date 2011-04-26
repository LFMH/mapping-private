#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_artificial
n=0

mkdir -p $DATA/features_c
mkdir -p $DATA/features_d
mkdir -p $DATA/features_g
mkdir -p $DATA/features_r

for i in `find $DATA/data -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
do
    echo "Processing $i"
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeFeature $i c -rotate 1 -subdiv 7 -offset 2 $DATA $DATA/features_c/$num.pcd	
    rosrun color_feature_classification computeFeature $i d -rotate 1 -subdiv 7 -offset 2 $DATA $DATA/features_d/$num.pcd	
    rosrun color_feature_classification computeFeature $i g -rotate 1 -subdiv 7 -offset 2 $DATA $DATA/features_g/$num.pcd	
    rosrun color_feature_classification computeFeature $i r -rotate 1 -subdiv 7 -offset 1 $DATA $DATA/features_r/$num.pcd	
    n=`expr $n + 1`	
done