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
    mkdir -p $DATA/training_features_g/$dir_name

    n=0
    for j in `find $DATA/training/$dir_name -type f -iname "*.pcd"`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeGRSD $j -rotate 1 -subdiv $subdiv -offset $offset $DIR $DATA/training_features_g/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done

for dir_name in `ls $DATA/testing`
do
    mkdir -p $DATA/testing_features_g/$dir_name

    n=0
    for j in `find $DATA/testing/$dir_name -type f -iname "*.pcd"`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeGRSD $j $DIR $DATA/testing_features_g/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done

for dir_name in `ls $DATA/unseen`
do
    mkdir -p $DATA/unseen_features_g/$dir_name

    n=0
    for j in `find $DATA/unseen/$dir_name -type f -iname "*.pcd"`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeGRSD $j $DIR $DATA/unseen_features_g/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done