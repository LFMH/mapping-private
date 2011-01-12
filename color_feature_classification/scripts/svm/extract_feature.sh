#!/bin/bash

# initialize class label
class=0

# go through all files
for pcd in `ls $1/*.pcd`
do
  # increment class label
  ((class++))
  # get the last line, containing the feature:
  feature=`tail -n 1 $pcd`
  echo $feature > $1/$class.feature
  # write the feature and class label in svm format
  svm=`awk '/^[0-9+.-]/{printf("%d",'$class'); for (i=1; i<='$2'; i++) printf(" %d:%s",i,$i); printf("\n");}' $1/$class.feature`
  echo $svm >> $3
  #tail -n 1 $3
done
echo "Total classes: "$class

