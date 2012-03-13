#!/bin/bash
# Example directory containing _vfh.pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/rgbd-sel-smooth-reduced-arranged-noball6

if [ $# != 2 ]
then
    echo "usage: bash `rospack find color_feature_classification`/scripts/demos_grsd/2.test_classify.sh [feature_type (e.g. g)] [dim_of_subspace (e.g. 18)]"
    exit;
fi

feature=$1
sub=$2

# NOTE: comment-out the followings if you don't use normalization
norm_flag="-norm $DIR/bin_normalization/max_$feature.txt"

for num in `ls $DIR/pca_result_g/`
do
    mkdir -p $DIR/data/obj$num
done

i=0
for in_dir_name in `ls $DATA/testing_features_${feature}/`
do
    class_num=`echo $in_dir_name | cut -c1-1`
    #class_num=`expr $class_num - 1`
    dir_name=$(printf "obj%03d" $class_num)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find $DATA/testing_features_${feature}/$in_dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file_GRSD $j $feature s -sub $sub $norm_flag $DIR
    done
    i=`expr $i + 1`
done
