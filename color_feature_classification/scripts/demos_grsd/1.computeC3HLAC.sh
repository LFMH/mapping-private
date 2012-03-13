#!/bin/bash
# Example directory containing .pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/rgbd-sel-smooth-reduced-arranged-noball6

subdiv=$1 #7 for classification experiment
offset=$2 #2 for classification experiment

mkdir -p $DATA/training_features_c
mkdir -p $DATA/testing_features_c

for dir_name in `ls $DATA/training`
do
    for dir_name2 in `ls $DATA/training/$dir_name`
    do
	mkdir -p $DATA/training_features_c/${dir_name}_$dir_name2

	n=0
	for j in `find $DATA/training/$dir_name/$dir_name2 -type f -iname "*.pcd"`
	do
	    echo "Processing $j"
	    num=$(printf "%05d" $n)
	    rosrun color_feature_classification computeFeature $j c -rotate 1 -subdiv $subdiv -offset $offset $DIR $DATA/training_features_c/${dir_name}_$dir_name2/$num.pcd	
	    n=`expr $n + 1`
	done
    done
done

for dir_name in `ls $DATA/testing`
do
    for dir_name2 in `ls $DATA/testing/$dir_name`
    do
	mkdir -p $DATA/testing_features_c/${dir_name}_$dir_name2

	n=0
	for j in `find $DATA/testing/$dir_name/$dir_name2 -type f -iname "*.pcd"`
	do
	    echo "Processing $j"
	    num=$(printf "%05d" $n)
	    rosrun color_feature_classification computeFeature $j c -rotate 1 $DIR $DATA/testing_features_c/${dir_name}_$dir_name2/$num.pcd	
	    n=`expr $n + 1`
	done
    done
done
