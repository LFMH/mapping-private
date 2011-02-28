#!/bin/bash
DATA=`rospack find color_feature_classification`/demos_artificial

mkdir normalized_histograms

for shape in cone cube cylinder dice plane sphere torus
do
    for color in black blue green orange purple red yellow
    do
	rosrun color_feature_classification computeFeature $DATA/data/obj_${shape}_${color}.pcd r $DATA normalized_histograms/noiseless_${shape}_${color}.pcd
	rosrun color_feature_classification normalizeHist normalized_histograms/noiseless_${shape}_${color}.pcd $DATA/bin_normalization/max_r.txt normalized_histograms/noiseless_${shape}_${color}.pcd
    done
done
