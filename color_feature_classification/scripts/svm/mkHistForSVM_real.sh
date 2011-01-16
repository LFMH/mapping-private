#!/bin/bash
# Example directory containing .pcd files
DEMO=`rospack find color_feature_classification`/demos
DATA=${DEMO}/data/
n=0

bash `rospack find color_feature_classification`/scripts/demos/2.computeFeature.sh

mkdir hist_data_forSVM_real
cp -r $DEMO/test_features_c/ hist_data_forSVM_real/train_features_c
cp -r $DEMO/test_features_d/ hist_data_forSVM_real/train_features_d
cp -r $DEMO/test_features_g/ hist_data_forSVM_real/train_features_g
cp -r $DEMO/test_features_r/ hist_data_forSVM_real/train_features_r