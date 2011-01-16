#!/bin/bash

DATA=`rospack find color_feature_classification`/demos
dirNum=`ls $DATA/data/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    #echo $dir_name
    echo `ls $DATA/data/$dir_name/*000.pcd` >> object_names.txt
done


