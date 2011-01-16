#!/bin/bash

# reset training file
rm $3_test_$4.svm
#echo "Saving results to: $3_test_$4.svm"

# initialize class label
class=0

# go through all files
for obj in $(find $1/obj* -type d)
do
  # increment class label
  ((class++))
  #for pcd in $obj/*.pcd
  for pcd in `find $obj -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
  do
    # get the last line, containing the feature:
    feature=`tail -n 1 $pcd`
    echo $feature > $obj/$class.feature
    # write the feature and class label in svm format
    svm=`awk '/^[0-9+.-]/{printf("%d",'$class'); for (i=1; i<='$2'; i++) printf(" %d:%s",i,$i); printf("\n");}' $obj/$class.feature`
    echo $svm >> $3_test_$4.svm
    #tail -n 1 $3_test_$4.svm
  done
done
#echo "Total classes: "$class

# scale data
`rospack find libsvm`/build/libsvm-3.0/svm-scale -r $3_train.scale $3_test_$4.svm > $3_test_$4_scaled.svm

# test model
`rospack find libsvm`/build/libsvm-3.0/svm-predict $3_test_$4_scaled.svm $3_train_scaled.model $3_test_$4_scaled.result
#echo "Result written to: $3_test_$4_scaled.result"
#cat $3_test_$4_scaled.result
#echo "Number of predicted classes: "
#cat $3_test_$4_scaled.result | sort | uniq | wc -w

