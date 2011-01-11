#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

dirNum=`ls $DATA/data/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    #echo $dir_name
    mkdir $DATA/features_c/$dir_name
    mkdir $DATA/features_d/$dir_name
    mkdir $DATA/features_g/$dir_name
    mkdir $DATA/features_r/$dir_name

    n=0
    for j in `find $DATA/data/$dir_name -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeFeature $j c -rotate 1 -subdiv 10 -offset 5 $DATA/features_c/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j d -rotate 1 -subdiv 10 -offset 5 $DATA/features_d/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j g -subdiv 10 -offset 5 $DATA/features_g/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j r -subdiv 10 -offset 5 $DATA/features_r/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done