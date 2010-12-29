#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data/
n=0

#for i in `find $DATA -type f \( -iname "noisy_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -R`
for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -R`
do
    echo $i
    num=$(printf "%03d" $n)
    dir_name=$(printf "obj%03d" $n)
    #echo $dir_name
    rosrun color_feature_classification test_classify $i c s -sub 30 -dim 100 -comp pca_result_c/compress_axis
    #rosrun color_feature_classification test_classify $i d s -sub 100 -dim 800 -comp pca_result_d/compress_axis
    #rosrun color_feature_classification test_classify $i g s -sub 10 
    #rosrun color_feature_classification test_classify $i r s -sub 80
    ### rosrun color_feature_classification test_classify $i r s -sub 10 -dim 50 -comp pca_result_r/compress_axis
    n=`expr $n + 1`
done