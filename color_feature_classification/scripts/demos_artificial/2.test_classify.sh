#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos_artificial

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_c.txt"
norm_flag_d="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_d.txt"
norm_flag_g="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_g.txt"
norm_flag_r="-norm `rospack find color_feature_classification`/demos_artificial/bin_normalization/max_r.txt"

echo "----------------------------------------------------------"
echo "colorCHLAC - rotateion variant - (981)"
echo "----------------------------------------------------------"
n=0
for i in `find $DATA/test_features_c/ -type f -iname "*.pcd" | sort -d`
do
    #echo $i
    #echo "----------------"
    echo $n
    rosrun color_feature_classification test_classify_from_file $i c s -sub 15 -dim 50 -comp $DATA/pca_result_c/compress_axis $norm_flag_c $DATA
    #rosrun color_feature_classification test_classify_from_file $i c s -sub 15 $norm_flag_c $DATA
    n=`expr $n + 1`
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
n=0
for i in `find $DATA/test_features_d/ -type f -iname "*.pcd" | sort -d`
do
    #echo $i
    #echo "----------------"
    echo $n
    rosrun color_feature_classification test_classify_from_file $i d s -sub 15 -dim 50 -comp $DATA/pca_result_d/compress_axis $norm_flag_d $DATA
    #rosrun color_feature_classification test_classify_from_file $i d s -sub 15 $norm_flag_d $DATA
    n=`expr $n + 1`
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
n=0
for i in `find $DATA/test_features_g/ -type f -iname "*.pcd" | sort -d`
do
    #echo $i
    #echo "----------------"
    echo $n
    rosrun color_feature_classification test_classify_from_file $i g s -sub 15 $norm_flag_g $DATA
    n=`expr $n + 1`
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
n=0
for i in `find $DATA/test_features_r/ -type f -iname "*.pcd" | sort -d`
do
    #echo $i
    #echo "----------------"
    echo $n
    rosrun color_feature_classification test_classify_from_file $i r s -sub 15 -dim 50 -comp $DATA/pca_result_r/compress_axis $norm_flag_r $DATA
    #rosrun color_feature_classification test_classify_from_file $i r s -sub 15 $norm_flag_r $DATA
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