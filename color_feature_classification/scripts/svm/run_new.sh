#!/bin/bash

# TODO: read number of features from file

#GRSD 32.0 0.5 72.1561
#CHLAC 32.0 0.03125 99.6032
#CONCAT 128.0 0.0078125 99.6032
#VOSCH 512.0 0.0078125 98.9418
#VOSCH_plus 512.0 0.0078125 97.619

echo ""; echo "========== TRAIN =========="

echo ""; echo "---------- GRSD ---------"
#bash train_svm.sh $2/train_features_g 20 $1_grsd 128 0.5
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_g 20 $1_grsd 512 0.5
echo ""; echo "---------- CHLAC ---------"
#bash train_svm.sh $2/train_features_c 981 $1_chlac 128 0.0078125
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_c 981 $1_chlac 32 0.03125
echo ""; echo "---------- CONCAT ---------"
#bash train_svm.sh $2/train_features_d 1001 $1_concat 128 0.0078125
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_d 1001 $1_concat 128 0.0078125
echo ""; echo "---------- VOSCH ---------"
#bash train_svm.sh $2/train_features_r 137 $1_vosch 128 0.007812
bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_r 137 $1_vosch 512 0.0078125
# echo ""; echo "---------- VOSCH Plus ---------"
# bash `rospack find color_feature_classification`/scripts/svm/train_svm.sh $2/train_features_r_plus 227 $1_vosch_plus 512 0.0078125

#levels=() #("0.0005" "0.0010" "0.0015" "0.0020" "0.0025" "0.0030" "0.0035" "0.0040" "0.0045" "0.0050")
#for noise in ${levels[@]}
#do
#  echo ""; echo "========== TEST $noise =========="
#  base=$2/test_features_$noise
#  for rotation in 0 # 1
#  do
#    echo ""; echo "---------- GRSD $noise / $rotation ----------"
#    bash test_svm.sh $rotation/test_features_g 20 $1_grsd $base $noise
#    echo ""; echo "---------- CHLAC $noise / $rotation ----------"
#    bash test_svm.sh $rotation/test_features_c 981 $1_chlac $base $noise
#    echo ""; echo "---------- CONCAT $noise / $rotation ----------"
#    bash test_svm.sh $rotation/test_features_d 1001 $1_concat $base $noise
#    echo ""; echo "---------- VOSCH $noise / $rotation ----------"
#    bash test_svm.sh $rotation/test_features_r 137 $1_vosch $base $noise
#  done
#done

n=0
for dir in `ls $2/novel_view/* -d`
do
  echo ""; echo "========== TEST $dir =========="
  echo ""; echo "---------- GRSD $dir ----------"
  bash `rospack find color_feature_classification`/scripts/svm/test_svm_novel.sh $dir/test_features_g 20 $1_grsd $n
  echo ""; echo "---------- CHLAC $dir ----------"
  bash `rospack find color_feature_classification`/scripts/svm/test_svm_novel.sh $dir/test_features_c 981 $1_chlac $n
  echo ""; echo "---------- CONCAT $dir ----------"
  bash `rospack find color_feature_classification`/scripts/svm/test_svm_novel.sh $dir/test_features_d 1001 $1_concat $n
  echo ""; echo "---------- VOSCH $dir ----------"
  bash `rospack find color_feature_classification`/scripts/svm/test_svm_novel.sh $dir/test_features_r_old 137 $1_vosch $n
  # echo ""; echo "---------- VOSCH Plus $dir ----------"
  # bash `rospack find color_feature_classification`/scripts/svm/test_svm_novel.sh $dir/test_features_r 227 $1_vosch_plus $n
  ((n++))
done

