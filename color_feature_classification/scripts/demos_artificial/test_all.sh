#!/bin/bash

DATA=`rospack find color_feature_classification`/demos_artificial

for rot in 0 1
do
    mkdir -p $DATA/data_result_$rot
    for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
    do
	for x in a #b c d e
	do
	    bash `rospack find color_feature_classification`/scripts/demos_artificial/2.test_classify.sh $DATA/hist_data_forSVM/test_features_${nlevel}${x}_$rot > $DATA/data_result_$rot/result_${nlevel}_${x}.txt
	done
    done
done

