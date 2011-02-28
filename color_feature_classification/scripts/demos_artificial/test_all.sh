#!/bin/bash

# $1 = directory name

for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
do
    for x in a #b c d e
    do
    # copy the test shape data
	for shape in cone cube cylinder dice plane sphere torus
	do
	    for color in black blue green orange purple red yellow
	    do
		cp $1/noisy${nlevel}${x}_${shape}_${color}.pcd data/noisy_${shape}_${color}.pcd
		rosrun color_chlac convertOld2NewPCD data/noisy_${shape}_${color}.pcd data/noisy_${shape}_${color}.pcd
	    done
	done
	
              # compute test histograms
	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh 1
	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.test_classify.sh > data_result/result_${nlevel}_${x}.txt
    done
done

