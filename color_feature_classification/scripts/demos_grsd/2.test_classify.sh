#!/bin/bash
# Example directory containing _vfh.pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/semantic-3d-geometric

sub=$1

# NOTE: comment-out the followings if you don't use normalization
norm_flag_g="-norm $DIR/bin_normalization/max_g.txt"

for num in `ls $DIR/pca_result_g/`
do
    mkdir -p data/obj$num
done

i=0
#for in_dir_name in `ls $DATA/testing_features_g/`
for in_dir_name in `ls $DATA/test_training_features_g/`
#for in_dir_name in `ls $DATA/unseen_features_g/`
do
    #dir_name=$(printf "obj%03d" $i)
    class_num=`echo $in_dir_name | cut -c1-1`
    #class_num=`expr $class_num - 1`
    dir_name=$(printf "obj%03d" $class_num)
    echo $dir_name
    echo "---------------------------------------------------------"
    #for j in `find $DATA/testing_features_g/$in_dir_name -type f -iname "*.pcd" | sort -d`
    for j in `find $DATA/test_training_features_g/$in_dir_name -type f -iname "*.pcd" | sort -d`
    #for j in `find $DATA/unseen_features_g/$in_dir_name -type f -iname "*.pcd" | sort -d`
    do
	#rosrun color_feature_classification test_classify_from_file_GRSD $j g s -sub $sub -dim 100 -comp $DIR/pca_result_g/compress_axis $norm_flag_g $DIR
	rosrun color_feature_classification test_classify_from_file_GRSD $j g s -sub $sub $norm_flag_g $DIR
    done
    i=`expr $i + 1`
done
