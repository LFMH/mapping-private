#!/bin/bash
# Example directory containing .pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric

mkdir -p $DATA/test_training_features_g

for dir_name in `ls $DATA/training`
do
    mkdir -p $DATA/test_training_features_g/$dir_name

    n=0
    for j in `find $DATA/training/$dir_name -type f -iname "*.pcd"`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeGRSD $j $DIR $DATA/test_training_features_g/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done
