#!/bin/bash

# $1 = directory name

# for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
# do
#     for x in a b c d e
#     do
#     # copy the test shape data
# 	for shape in cone cube cylinder dice plane sphere torus
# 	do
# 	    for color in black blue green orange purple red yellow
# 	    do
# 		cp $1/noisy${nlevel}${x}_${shape}_${color}.pcd data/noisy_${shape}_${color}.pcd
# 		rosrun color_feature_classification convertOld2NewPCD data/noisy_${shape}_${color}.pcd data/noisy_${shape}_${color}.pcd
# 	    done
# 	done
	
# 	mkdir -p hist_data_forSVM/test_features_${nlevel}${x}_1/
# 	mkdir -p hist_data_forSVM/test_features_${nlevel}${x}_0/

#               # compute test histograms
# 	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh 1
# 	mv test_features_c hist_data_forSVM/test_features_${nlevel}${x}_1/
# 	mv test_features_d hist_data_forSVM/test_features_${nlevel}${x}_1/
# 	mv test_features_g hist_data_forSVM/test_features_${nlevel}${x}_1/
# 	mv test_features_r hist_data_forSVM/test_features_${nlevel}${x}_1/
# 	bash `rospack find color_feature_classification`/scripts/demos_artificial/2.computeFeature.sh 0
# 	mv test_features_c hist_data_forSVM/test_features_${nlevel}${x}_0/
# 	mv test_features_d hist_data_forSVM/test_features_${nlevel}${x}_0/
# 	mv test_features_g hist_data_forSVM/test_features_${nlevel}${x}_0/
# 	mv test_features_r hist_data_forSVM/test_features_${nlevel}${x}_0/

# 	#bash `rospack find color_feature_classification`/scripts/demos_artificial/2.test_classify.sh > data_result/result_${nlevel}_${x}.txt
#     done
# done

for rot in 0 1
do
    mkdir data_result_$rot
    for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
    do
	for x in a #b c d e
	do
	    rm test_features_c
	    rm test_features_d
	    rm test_features_g
	    rm test_features_r
	    ln -s hist_data_forSVM/test_features_${nlevel}${x}_$rot/test_features_c test_features_c
	    ln -s hist_data_forSVM/test_features_${nlevel}${x}_$rot/test_features_d test_features_d
	    ln -s hist_data_forSVM/test_features_${nlevel}${x}_$rot/test_features_g test_features_g
	    ln -s hist_data_forSVM/test_features_${nlevel}${x}_$rot/test_features_r test_features_r
	    bash `rospack find color_feature_classification`/scripts/demos_artificial/2.test_classify.sh > data_result_$rot/result_${nlevel}_${x}.txt
	done
    done
done

