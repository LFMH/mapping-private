#!/bin/bash

# $1 = directory name

DATA=`rospack find color_feature_classification`/demos_artificial
mkdir -p $DATA/hist_data_forSVM

for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
do
    for x in a b c d e
    do
    # copy the test shape data
	for shape in cone cube cylinder dice plane sphere torus
	do
	    for color in black blue green orange purple red yellow
	    do
		cp $1/noisy${nlevel}${x}_${shape}_${color}.pcd $DATA/data/noisy_${shape}_${color}.pcd
		rosrun color_feature_classification convertOld2NewPCD $DATA/data/noisy_${shape}_${color}.pcd $DATA/data/noisy_${shape}_${color}.pcd
	    done
	done
	
	mkdir -p $DATA/hist_data_forSVM/test_features_${nlevel}${x}_1/
	mkdir -p $DATA/hist_data_forSVM/test_features_${nlevel}${x}_0/

              # compute test histograms
	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh 1
	mv $DATA/test_features_c $DATA/hist_data_forSVM/test_features_${nlevel}${x}_1/
	mv $DATA/test_features_d $DATA/hist_data_forSVM/test_features_${nlevel}${x}_1/
	mv $DATA/test_features_g $DATA/hist_data_forSVM/test_features_${nlevel}${x}_1/
	mv $DATA/test_features_r $DATA/hist_data_forSVM/test_features_${nlevel}${x}_1/
	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh 0
	mv $DATA/test_features_c $DATA/hist_data_forSVM/test_features_${nlevel}${x}_0/
	mv $DATA/test_features_d $DATA/hist_data_forSVM/test_features_${nlevel}${x}_0/
	mv $DATA/test_features_g $DATA/hist_data_forSVM/test_features_${nlevel}${x}_0/
	mv $DATA/test_features_r $DATA/hist_data_forSVM/test_features_${nlevel}${x}_0/
    done
done