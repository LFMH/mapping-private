#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

for i in `find $DATA -type d -name "*"`
do
    if [ $DATA != $i ]
    then
	echo $i
	echo "---------------------------------------------------------"
	for j in `find $i -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
	do
	    #rosrun color_feature_classification test_classify $j c s -sub 100 -dim 800 -comp pca_result_c/compress_axis
	    #rosrun color_feature_classification test_classify $j d s -sub 100 -dim 800 -comp pca_result_d/compress_axis
	    #rosrun color_feature_classification test_classify $j g s -sub 10 
	    rosrun color_feature_classification test_classify $j r s -sub 80
	    ##### rosrun color_feature_classification test_classify $j r s -sub 10 -dim 50 -comp pca_result_r/compress_axis
	done
    fi
done