#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data/
n=0

mkdir hist_data_forSVM
mkdir hist_data_forSVM/train_features_c
mkdir hist_data_forSVM/train_features_d
mkdir hist_data_forSVM/train_features_g
mkdir hist_data_forSVM/train_features_r

for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
do
    echo $i
    num=$(printf "%03d" $n)
    mkdir hist_data_forSVM/train_features_c/obj$num
    mkdir hist_data_forSVM/train_features_d/obj$num
    mkdir hist_data_forSVM/train_features_g/obj$num
    mkdir hist_data_forSVM/train_features_r/obj$num
    rosrun color_feature_classification computeFeatureForSVM $i c -rotate 1 hist_data_forSVM/train_features_c/obj$num
    rosrun color_feature_classification computeFeatureForSVM $i d -rotate 1 hist_data_forSVM/train_features_d/obj$num
    rosrun color_feature_classification computeFeatureForSVM $i g -rotate 1 hist_data_forSVM/train_features_g/obj$num
    rosrun color_feature_classification computeFeatureForSVM $i r -rotate 1 hist_data_forSVM/train_features_r/obj$num
    n=`expr $n + 1`
done
