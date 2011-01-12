#!/bin/bash

# $1 = directory name
# Usage: "bash ../scripts/mkHistForSVM_all.sh [noisy_shape directory name]"  in color_feature_classification/demos_artificial
# Note that there should be 49 train shape data with name "obj_cone_black.pcd" etc. in data/ 

DATA=`rospack find color_feature_classification`/demos_artificial

rotate=$2
[ -z $2 ] && rotate=1

mkdir hist_data_forSVM
for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
do
    for x in a b c d e
    do
        # copy and convert the test shape data
	for shape in cone cube cylinder dice plane sphere torus
	do
	    for color in black blue green orange purple red yellow
	    do
		cp $1/noisy${nlevel}${x}_${shape}_${color}.pcd $DATA/data/noisy_${shape}_${color}.pcd
		rosrun color_chlac convertOld2NewPCD $DATA/data/noisy_${shape}_${color}.pcd $DATA/data/noisy_${shape}_${color}.pcd
	    done
	done
	
        # compute test histograms
	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh $rotate

	mkdir hist_data_forSVM/test_features_${nlevel}${x}
	mv $DATA/test_features_c hist_data_forSVM/test_features_${nlevel}${x}_$rotate
	mv $DATA/test_features_d hist_data_forSVM/test_features_${nlevel}${x}_$rotate
	mv $DATA/test_features_g hist_data_forSVM/test_features_${nlevel}${x}_$rotate
	mv $DATA/test_features_r hist_data_forSVM/test_features_${nlevel}${x}_$rotate
	#mkdir test_features_c
	#mkdir test_features_d
	#mkdir test_features_g
	#mkdir test_features_r
    done
done

