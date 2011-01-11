#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data
n=0

for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
do
    echo "Processing $i"
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeFeature $i c -rotate 1 -subdiv 5 -offset 2 features_c/$num.pcd	
    rosrun color_feature_classification computeFeature $i d -rotate 2 -subdiv 5 -offset 2 features_d/$num.pcd	
    #rosrun color_feature_classification computeFeature $i g -subdiv 5 -offset 2 features_g/$num.pcd	
    rosrun color_feature_classification computeFeature $i r -rotate 2 -subdiv 5 -offset 2 features_r/$num.pcd	
    n=`expr $n + 1`	
done