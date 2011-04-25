#!/bin/bash
# Example directory containing .pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric

subdiv=$1 #7 for classification experiment
offset=$2 #2 for classification experiment

mkdir -p $DATA/training_features_g
mkdir -p $DATA/testing_features_g
mkdir -p $DATA/unseen_features_g

for dir_name in `ls $DATA/training`
do
    for dir_name2 in `ls $DATA/training/$dir_name`
    do
	mkdir -p $DATA/training_features_g/${dir_name}_$dir_name2

	n=0
	for j in `find $DATA/training/$dir_name/$dir_name2 -type f -iname "*.pcd"`
	do
	    echo "Processing $j"
	    num=$(printf "%03d" $n)
	    rosrun color_feature_classification computeGRSD $j -rotate 1 -subdiv $subdiv -offset $offset $DIR $DATA/training_features_g/${dir_name}_$dir_name2/$num.pcd	
	    n=`expr $n + 1`	
	done
    done
done
