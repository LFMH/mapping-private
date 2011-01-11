#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`pwd`/data

echo "----------------------------------------------------------"
echo "colorCHLAC - rotateion variant - (981)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j c s -sub 5 -dim 100 -comp pca_result_c/compress_axis
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_d/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j d s -sub 5 -dim 100 -comp pca_result_d/compress_axis
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_g/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j g s -sub 5
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_r/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j r s -sub 5 -dim 80 -comp pca_result_r/compress_axis
    done
done


# dirNum=`ls data/* -d | wc -l`
# for((i=0;i<$dirNum;i++))
# do
#     dir_name=$(printf "obj%03d" $i)
#     echo $DATA/$dir_name
#     echo "---------------------------------------------------------"
#     for j in `find $DATA/$dir_name -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
#     do
# 	#rosrun color_feature_classification test_classify $j c s -sub 100 -dim 800 -comp pca_result_c/compress_axis
# 	#rosrun color_feature_classification test_classify $j d s -sub 100 -dim 800 -comp pca_result_d/compress_axis
# 	#rosrun color_feature_classification test_classify $j d s -sub 30 -dim 100 -comp pca_result_d/compress_axis
# 	#rosrun color_feature_classification test_classify $j g s -sub 10 
# 	#rosrun color_feature_classification test_classify $j r s -sub 80
# 	rosrun color_feature_classification test_classify $j r s -sub 40 -dim 100 -comp pca_result_r/compress_axis
#     done
# done