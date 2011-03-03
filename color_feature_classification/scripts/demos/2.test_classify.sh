#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`rospack find color_feature_classification`/demos

sub=$1

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

echo "----------------------------------------------------------"
echo "colorCHLAC - rotation variant - (981)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find $DATA/test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j c s -sub $sub -dim 100 -comp $DATA/pca_result_c/compress_axis $norm_flag_c
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_d/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find $DATA/test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j d s -sub $sub -dim 100 -comp $DATA/pca_result_d/compress_axis $norm_flag_d
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_g/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find $DATA/test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	echo "-1 0"
	#rosrun color_feature_classification test_classify_from_file $j g s -sub $sub $norm_flag_g
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_r/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    for j in `find $DATA/test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`
    do
	rosrun color_feature_classification test_classify_from_file $j r s -sub $sub -dim 100 -comp $DATA/pca_result_r/compress_axis $norm_flag_r
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