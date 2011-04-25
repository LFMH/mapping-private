#!/bin/bash

# values I used for all synthetic data
cost=$4
[ -z $4 ] && cost=1000
gamma=$5
[ -z $5 ] && gamma=0.00078125

# reset training file
rm $3_train.svm
echo "Saving results to: $3_train.svm"

# initialize class label
class=0

# go through all files
for obj in $(find $1/obj* -type d)
do
  # increment class label
  ((class++))
  for pcd in $obj/*.pcd
  do
    # get the last line, containing the feature:
    feature=`tail -n 1 $pcd`
    echo $feature > $obj/$class.feature
    # write the feature and class label in svm format
    svm=`awk '/^[0-9+.-]/{printf("%d",'$class'); for (i=1; i<='$2'; i++) printf(" %d:%s",i,$i); printf("\n");}' $obj/$class.feature`
    echo $svm >> $3_train.svm
    #tail -n 1 $3_train.svm
  done
done
echo "Total classes: "$class

# scale data
`rospack find libsvm`/build/libsvm-3.0/svm-scale -s $3_train.scale $3_train.svm > $3_train_scaled.svm

# create model
#echo "`rospack find libsvm`/build/libsvm-3.0/svm-train -s 0 -c $cost -g $gamma $3_train_scaled.svm $3_train_scaled.model > $3_train_scaled.output"
`rospack find libsvm`/build/libsvm-3.0/svm-train -s 0 -c $cost -g $gamma $3_train_scaled.svm $3_train_scaled.model > $3_train_scaled.output
echo "Output written to: $3_train_scaled.output"

# test model
`rospack find libsvm`/build/libsvm-3.0/svm-predict $3_train_scaled.svm $3_train_scaled.model $3_train_scaled.result
echo "Result written to: $3_train_scaled.result"
#cat $3_train_scaled.result
echo "Number of predicted classes: "
cat $3_train_scaled.result | sort | uniq | wc -w

