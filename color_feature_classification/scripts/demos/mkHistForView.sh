#!/bin/bash
# Usage: e.g.  $ bash mkHistForView.sh obj002

DATA=`rospack find color_feature_classification`/demos

dir_name=$1
mkdir normalized_histograms_$dir_name

files=(`ls $DATA/data/$dir_name`)
num=`echo ${#files[@]}`
for((i=0;i<$num;i++))
do
    rosrun color_feature_classification computeFeature $DATA/data/$dir_name/${files[$i]} r $DATA normalized_histograms_$dir_name/${files[$i]}
    rosrun color_feature_classification normalizeHist normalized_histograms_$dir_name/${files[$i]} $DATA/bin_normalization/minmax_r.txt normalized_histograms_$dir_name/${files[$i]}
done
