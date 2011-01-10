#!/bin/bash

# training file
echo ""; echo "---------- TRAIN ---------"
rm $3_train.svm
./extract_feature.sh train_features_$1 $2 $3_train.svm

# testing file
echo ""; echo "---------- TEST ---------"
rm $3_test.svm
./extract_feature.sh test_features_$1 $2 $3_test.svm

# TODO: read number of features from file
# TODO: use scaling?
# TODO: get best c and g value (more training and easy.py?)

# create model
echo ""; echo "---------- MODEL ---------"
`rospack find libsvm`/build/libsvm-3.0/svm-train -s 0 -c 0.03125 -g 0.0078125 $3_train.svm $3_train.model > $3_train.output
echo "Output written to: "$3"_train.output"

# test model
echo ""; echo "---------- PREDICT ---------"
`rospack find libsvm`/build/libsvm-3.0/svm-predict $3_test.svm $3_train.model $3_test.result
cat $3_test.result
echo "Number of predicted classes: "
cat $3_test.result | sort | uniq | wc -w

