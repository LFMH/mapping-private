#!/bin/bash
# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

dirNum=`ls $DATA/data/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    #echo $dir_name
    mkdir $DATA/test_features_c/$dir_name
    mkdir $DATA/test_features_d/$dir_name
    mkdir $DATA/test_features_g/$dir_name
    mkdir $DATA/test_features_r/$dir_name

    n=0
    for j in `find $DATA/data/$dir_name -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeFeature $j c $DATA/test_features_c/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j d $DATA/test_features_d/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j g $DATA/test_features_g/$dir_name/$num.pcd	
	rosrun color_feature_classification computeFeature $j r $DATA/test_features_r/$dir_name/$num.pcd	
	n=`expr $n + 1`	
    done
done

# n=0
# #for i in `find $DATA -type f \( -iname "noisy_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
# for i in `find $DATA -type f \( -iname "obj_*.pcd" ! -iname "*GRSD_CCHLAC.pcd" ! -iname "*colorCHLAC.pcd" \) | sort -d`
# do
#     echo $i
#     num=$(printf "%03d" $n)
#     dir_name=$(printf "obj%03d" $n)
#     #echo $dir_name
#     rosrun color_feature_classification computeFeature $i c test_features_c/$num.pcd
#     n=`expr $n + 1`
# done