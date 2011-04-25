#!/bin/bash

# TODO: read number of features from file

echo ""; echo "========== TRAIN =========="

echo ""; echo "---------- GRSD ---------"
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_g 20 $1_grsd
echo ""; echo "---------- CHLAC ---------"
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_c 981 $1_chlac
echo ""; echo "---------- CONCAT ---------"
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_d 1001 $1_concat
echo ""; echo "---------- VOSCH ---------"
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_r 227 $1_vosch

levels=("0.0005" "0.0010" "0.0015" "0.0020" "0.0025" "0.0030" "0.0035" "0.0040" "0.0045" "0.0050")
for noise in ${levels[@]}
do
 echo ""; echo "========== TEST $noise =========="
 base=$2/test_features_$noise
 for rotation in 0 1
 do
   echo ""; echo "---------- GRSD $noise / $rotation ----------"
   bash `rospack find color_feature_classification`/scripts/svm/test_svm.sh $rotation/test_features_g 20 $1_grsd $base $noise
   echo ""; echo "---------- CHLAC $noise / $rotation ----------"
   bash `rospack find color_feature_classification`/scripts/svm/test_svm.sh $rotation/test_features_c 981 $1_chlac $base $noise
   echo ""; echo "---------- CONCAT $noise / $rotation ----------"
   bash `rospack find color_feature_classification`/scripts/svm/test_svm.sh $rotation/test_features_d 1001 $1_concat $base $noise
   echo ""; echo "---------- VOSCH $noise / $rotation ----------"
   bash `rospack find color_feature_classification`/scripts/svm/test_svm.sh $rotation/test_features_r 227 $1_vosch $base $noise
 done
done
