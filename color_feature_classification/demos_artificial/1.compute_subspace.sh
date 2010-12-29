#!/bin/bash
# Example directory containing .pcd files
DATA=`pwd`/data/
n=0

# compute a subspace
for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -R`
do
    echo $i
    num=$(printf "%03d" $n)
    dir_name=$(printf "obj%03d" $n)
    #echo $dir_name
    rosrun color_feature_classification computeSubspace_with_rotate c $i -dim 100 -comp pca_result_c/compress_axis -rotate 1 -subdiv 5 pca_result_c/$num
    #rosrun color_feature_classification computeSubspace_with_rotate d $i -dim 800 -comp pca_result_d/compress_axis -rotate 1 -subdiv 5 pca_result_d/$num
    #rosrun color_feature_classification computeSubspace g $i -subdiv 5 pca_result_g/$num
    #rosrun color_feature_classification computeSubspace r $i -subdiv 5 pca_result_r/$num
    n=`expr $n + 1`
    #rosrun color_feature_classification computeSubspace r $files -dim 50 -comp pca_result_r/compress_axis pca_result_r/$num
done