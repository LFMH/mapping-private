#!/bin/bash
# Example directory containing .pcd files
DIR=`rospack find color_feature_classification`/demos_grsd
DATA=`rospack find color_feature_classification`/demos_grsd/data/rgbd-sel-smooth-reduced-arranged-noball6

# NOTE: comment-out the followings if you don't use normalization
norm_flag_g="-norm $DIR/bin_normalization/max_g.txt"
norm_flag_c="-norm $DIR/bin_normalization/max_c.txt"
norm_flag_r="-norm $DIR/bin_normalization/max_r.txt"

mkdir -p $DIR/training_features_g
mkdir -p $DIR/pca_result_g
mkdir -p $DIR/training_features_c
mkdir -p $DIR/pca_result_c
mkdir -p $DIR/training_features_r
mkdir -p $DIR/pca_result_r

i=0
for in_dir_name in `ls $DATA/training_features_g/`
do
    class_num=`echo $in_dir_name | cut -c1-1`
    class_num=`expr $class_num - 1`

    num=$(printf "%03d_%03d" $class_num $i)
    dir_name=$(printf "obj%03d_%03d" $class_num $i)
    ln -s $DATA/training_features_g/$in_dir_name $DIR/training_features_g/$dir_name
    ln -s $DATA/training_features_c/$in_dir_name $DIR/training_features_c/$dir_name
    ln -s $DATA/training_features_r/$in_dir_name $DIR/training_features_r/$dir_name
    echo $dir_name
    files=`find $DIR/training_features_g/$dir_name/ -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_g $DIR/pca_result_g/$num
    files=`find $DIR/training_features_c/$dir_name/ -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_c $DIR/pca_result_c/$num
    files=`find $DIR/training_features_r/$dir_name/ -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification computeSubspace_from_file $files $norm_flag_r $DIR/pca_result_r/$num
    i=`expr $i + 1`
done