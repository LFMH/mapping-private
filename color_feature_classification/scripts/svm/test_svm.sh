#!/bin/bash

# reset training file
rm $3_test_$5.svm
#echo "Saving results to: $3_test_$5.svm"

# go through all files
iterations=("a" "b" "c" "d" "e")
for i in ${iterations[@]}
do
  # initialize class label
  class=0

  obj=${4}${i}_${1}
  #echo $i" => "$obj
  for pcd in $obj/*.pcd
  do
    # increment class label
    ((class++))
    # get the last line, containing the feature:
    feature=`tail -n 1 $pcd`
    echo $feature > $obj/$class.feature
    # write the feature and class label in svm format
    svm=`awk '/^[0-9+.-]/{printf("%d",'$class'); for (i=1; i<='$2'; i++) printf(" %d:%s",i,$i); printf("\n");}' $obj/$class.feature`
    echo $svm >> $3_test_$5.svm
    #tail -n 1 $3_test_$5.svm
  done
done
#echo "Total classes: "$class

# scale data
`rospack find libsvm`/build/libsvm-3.0/svm-scale -r $3_train.scale $3_test_$5.svm > $3_test_$5_scaled.svm

# test model
`rospack find libsvm`/build/libsvm-3.0/svm-predict $3_test_$5_scaled.svm $3_train_scaled.model $3_test_$5_scaled.result
#echo "Result written to: $3_test_$5_scaled.result"
#cat $3_test_$5_scaled.result
#echo "Number of predicted classes: "
#cat $3_test_$5_scaled.result | sort | uniq | wc -w

