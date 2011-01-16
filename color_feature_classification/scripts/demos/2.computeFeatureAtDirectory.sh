#!/bin/bash

function Exit()
{
  echo Usage: $0 data_dir_name
  exit
}

# First parameter is mandatory
[ -z $1 ] && Exit

# The directory containing the configuration .txt files
DATA=`rospack find color_feature_classification`/demos

dirNum=`ls $1/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    #echo $dir_name

    if [ ! -d "$1/$dir_name" ]; then
      continue
    fi
    
    mkdir $1/test_features_c/$dir_name -p
    mkdir $1/test_features_d/$dir_name -p
    mkdir $1/test_features_g/$dir_name -p
    mkdir $1/test_features_r/$dir_name -p

    n=0
    for j in `find $1/$dir_name -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
    do
	echo "Processing $j"
	num=$(printf "%03d" $n)
	rosrun color_feature_classification computeFeature $j c $DATA $1/test_features_c/$dir_name/$num.pcd
	rosrun color_feature_classification computeFeature $j d $DATA $1/test_features_d/$dir_name/$num.pcd
	rosrun color_feature_classification computeFeature $j g $DATA $1/test_features_g/$dir_name/$num.pcd
	rosrun color_feature_classification computeFeature $j r $DATA $1/test_features_r/$dir_name/$num.pcd
	n=`expr $n + 1`
    done
done
