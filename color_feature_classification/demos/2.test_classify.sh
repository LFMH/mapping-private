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
	    #rosrun color_feature_classification test_classify $j v s

	    #rosrun color_feature_classification test_classify $j c s
	    rosrun color_feature_classification test_classify $j c s -dim 500 -comp pca_result_colorCHLAC/compress_axis
	    #rosrun color_feature_classification test_classify $j c k
	done
    fi
done