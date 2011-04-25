#!/bin/bash
# Example directory containing _vfh.pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric
#n=0

# NOTE: comment-out the followings if you don't use normalization
#norm_flag_g="-norm $DIR/bin_normalization/max_g.txt"

mkdir -p $DIR/projected_features

i=0
dir_num=`ls $DATA/test_training_features_g/* -d | wc -l`
dirs=(`ls $DATA/test_training_features_g/* -d`)
for in_dir_name in `ls $DATA/test_training_features_g/* -d`
do
    num=$(printf "%03d" $i)
    echo $in_dir_name
    dir_name=$(printf "obj%03d" $i)

    # the target class
    files=`find $in_dir_name/ -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification projectFeatures $files $norm_flag_g -comp2 $DIR/pca_result_g/$num $DIR/projected_features/$dir_name.txt

    # other classes
    files=""
    for((j=0;j<$dir_num;j++))
    do
	if [ $i -ne $j ]
	then
	    files="$files `find ${dirs[$j]}/ -type f -iname \"*.pcd\" | sort -d`"
	fi
    done
    rosrun color_feature_classification projectFeatures $files $norm_flag_g -comp2 $DIR/pca_result_g/$num $DIR/projected_features/${dir_name}_others.txt

    i=`expr $i + 1`
done
