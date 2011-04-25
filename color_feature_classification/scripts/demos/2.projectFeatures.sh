#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`rospack find color_feature_classification`/demos
#n=0

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

mkdir -p $DATA/projected_features_c
mkdir -p $DATA/projected_features_d
mkdir -p $DATA/projected_features_g
mkdir -p $DATA/projected_features_r

i=0
dirNum=`ls $DATA/test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name

########################
    # the target class
    files=`find $DATA/test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_c/compress_axis $norm_flag_c -comp2 $DATA/pca_result_c/$num $DATA/projected_features_c/$dir_name.txt

    # other classes
    dirs=(`ls $DATA/test_features_c/* -d`)
    files=""
    for((j=0;j<$dirNum;j++))
    do
	if [ $i -ne $j ]
	then
	    files="$files `find ${dirs[$j]}/ -type f -iname \"*.pcd\" | sort -d`"
	fi
    done
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_c/compress_axis $norm_flag_c -comp2 $DATA/pca_result_c/$num $DATA/projected_features_c/${dir_name}_others.txt
########################
    # the target class
    files=`find $DATA/test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_d/compress_axis $norm_flag_d -comp2 $DATA/pca_result_d/$num $DATA/projected_features_d/$dir_name.txt

    # other classes
    dirs=(`ls $DATA/test_features_d/* -d`)
    files=""
    for((j=0;j<$dirNum;j++))
    do
	if [ $i -ne $j ]
	then
	    files="$files `find ${dirs[$j]}/ -type f -iname \"*.pcd\" | sort -d`"
	fi
    done
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_d/compress_axis $norm_flag_d -comp2 $DATA/pca_result_d/$num $DATA/projected_features_d/${dir_name}_others.txt
########################
    # the target class
    files=`find $DATA/test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification projectFeatures $files $norm_flag_g -comp2 $DATA/pca_result_g/$num $DATA/projected_features_g/$dir_name.txt

    # other classes
    dirs=(`ls $DATA/test_features_g/* -d`)
    files=""
    for((j=0;j<$dirNum;j++))
    do
	if [ $i -ne $j ]
	then
	    files="$files `find ${dirs[$j]}/ -type f -iname \"*.pcd\" | sort -d`"
	fi
    done
    rosrun color_feature_classification projectFeatures $files $norm_flag_g -comp2 $DATA/pca_result_g/$num $DATA/projected_features_g/${dir_name}_others.txt
########################
    # the target class
    files=`find $DATA/test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_r/compress_axis $norm_flag_r -comp2 $DATA/pca_result_r/$num $DATA/projected_features_r/$dir_name.txt

    # other classes
    dirs=(`ls $DATA/test_features_r/* -d`)
    files=""
    for((j=0;j<$dirNum;j++))
    do
	if [ $i -ne $j ]
	then
	    files="$files `find ${dirs[$j]}/ -type f -iname \"*.pcd\" | sort -d`"
	fi
    done
    rosrun color_feature_classification projectFeatures $files -dim 100 -comp $DATA/pca_result_r/compress_axis $norm_flag_r -comp2 $DATA/pca_result_r/$num $DATA/projected_features_r/${dir_name}_others.txt
########################
done