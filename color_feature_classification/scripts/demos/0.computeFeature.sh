#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

subdiv=$1 #7 for classification experiment
offset=$2 #2 for classification experiment

dirNum=`ls $DATA/data/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    #echo $dir_name
    mkdir -p $DATA/features_c/$dir_name
    mkdir -p $DATA/features_d/$dir_name
    mkdir -p $DATA/features_g/$dir_name
    mkdir -p $DATA/features_r/$dir_name

    n=0
    for j in `find $DATA/data/$dir_name -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeFeature $j c -rotate 3 -subdiv $subdiv -offset $offset $DATA $DATA/features_c/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j d -rotate 3 -subdiv $subdiv -offset $offset $DATA $DATA/features_d/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j g -rotate 3 -subdiv $subdiv -offset $offset $DATA $DATA/features_g/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j r -rotate 3 -subdiv $subdiv -offset $offset $DATA $DATA/features_r/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done