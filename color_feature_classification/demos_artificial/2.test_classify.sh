#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data/

n=0
for i in `find test_features_c -type f -iname "*.pcd" | sort -d`
do
    echo $i
    echo $n
    rosrun color_feature_classification test_classify_from_file $i c s -sub 1 -dim 100 -comp pca_result_c/compress_axis
    n=`expr $n + 1`
done
#
n=0
for i in `find test_features_d -type f -iname "*.pcd" | sort -d`
do
    echo $i
    echo $n
    rosrun color_feature_classification test_classify_from_file $i d s -sub 1 -dim 100 -comp pca_result_d/compress_axis
    n=`expr $n + 1`
done
#
n=0
for i in `find test_features_g -type f -iname "*.pcd" | sort -d`
do
    echo $i
    echo $n
    rosrun color_feature_classification test_classify_from_file $i g s -sub 1
    n=`expr $n + 1`
done
#
n=0
for i in `find test_features_r -type f -iname "*.pcd" | sort -d`
do
    echo $i
    echo $n
    rosrun color_feature_classification test_classify_from_file $i r s -sub 1 -dim 100 -comp pca_result_r/compress_axis
    n=`expr $n + 1`
done

# #for i in `find $DATA -type f \( -iname "noisy_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
# for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
# do
#     echo $i
#     echo $n
#     num=$(printf "%03d" $n)
#     dir_name=$(printf "obj%03d" $n)
#     #echo $dir_name
#     rosrun color_feature_classification test_classify $i c s -sub 10 -dim 100 -comp pca_result_c/compress_axis
#     #rosrun color_feature_classification test_classify $i d s -sub 50 -dim 100 -comp pca_result_d/compress_axis
#     #rosrun color_feature_classification test_classify $i g s -sub 10 
#     #rosrun color_feature_classification test_classify $i r s -sub 80
#     ### rosrun color_feature_classification test_classify $i r s -sub 10 -dim 50 -comp pca_result_r/compress_axis
#     n=`expr $n + 1`
# done