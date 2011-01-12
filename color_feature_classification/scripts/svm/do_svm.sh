#!/bin/bash

# training file
echo ""; echo "---------- TRAIN ---------"
rm $3_train.svm
bash extract_feature.sh train_features_$1 $2 $3_train.svm

# testing file
echo ""; echo "---------- TEST ---------"
rm $3_test.svm
bash extract_feature.sh test_features_$1 $2 $3_test.svm

# TODO: read number of features from file

# scale data
`rospack find libsvm`/build/libsvm-3.0/svm-scale -s $3_train.scale $3_train.svm > $3_train_scaled.svm
`rospack find libsvm`/build/libsvm-3.0/svm-scale -r $3_train.scale $3_test.svm > $3_test_scaled.svm

# create model
echo ""; echo "---------- MODEL ---------"
`rospack find libsvm`/build/libsvm-3.0/svm-train -s 0 -c 32 -g 0.0078125 $3_train_scaled.svm $3_train_scaled.model > $3_train_scaled.output
echo "Output written to: "$3"_train_scaled.output"

# test model
echo ""; echo "---------- PREDICT ---------"
`rospack find libsvm`/build/libsvm-3.0/svm-predict $3_test_scaled.svm $3_train_scaled.model $3_test_scaled.result
#cat $3_test_scaled.result
echo "Number of predicted classes: "
cat $3_test_scaled.result | sort | uniq | wc -w

