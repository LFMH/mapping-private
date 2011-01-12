#!/bin/bash
DATA=`rospack find color_feature_classification`/demos_artificial
# compute a subspace
n=0
for i in `find $DATA/features_c -type f -iname "*.pcd" | sort -d`
do
    echo $i
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeSubspace_from_file $i -dim 100 -comp $DATA/pca_result_c/compress_axis $DATA/pca_result_c/$num
    n=`expr $n + 1`
done
#
n=0
for i in `find $DATA/features_d -type f -iname "*.pcd" | sort -d`
do
    echo $i
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeSubspace_from_file $i -dim 100 -comp $DATA/pca_result_d/compress_axis $DATA/pca_result_d/$num
    n=`expr $n + 1`
done
#
n=0
for i in `find $DATA/features_g -type f -iname "*.pcd" | sort -d`
do
    echo $i
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeSubspace_from_file $i $DATA/pca_result_g/$num
    n=`expr $n + 1`
done
#
n=0
for i in `find $DATA/features_r -type f -iname "*.pcd" | sort -d`
do
    echo $i
    num=$(printf "%03d" $n)
    rosrun color_feature_classification computeSubspace_from_file $i -dim 70 -comp $DATA/pca_result_r/compress_axis $DATA/pca_result_r/$num
    n=`expr $n + 1`
done

# # compute a subspace
# for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
# do
#     echo $i
#     num=$(printf "%03d" $n)
#     #dir_name=$(printf "obj%03d" $n)
#     #echo $dir_name
#     rosrun color_feature_classification computeSubspace_with_rotate c $i -dim 100 -comp pca_result_c/compress_axis -rotate 1 -subdiv 5 -offset 2 pca_result_c/$num
#     #rosrun color_feature_classification computeSubspace_with_rotate d $i -dim 100 -comp pca_result_d/compress_axis -rotate 1 -subdiv 5 -offset 2 pca_result_d/$num
#     #rosrun color_feature_classification computeSubspace g $i -subdiv 5 -offset 2 pca_result_g/$num
#     #rosrun color_feature_classification computeSubspace r $i -subdiv 5 -offset 2 pca_result_r/$num
#     n=`expr $n + 1`
#     #rosrun color_feature_classification computeSubspace r $files -dim 50 -comp pca_result_r/compress_axis pca_result_r/$num
# done