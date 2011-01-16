#!/bin/bash

function Exit()
{
  echo Usage: $0 nr_classes
  exit
}

# First parameter is mandatory
[ -z $1 ] && Exit

# Example directory containing .pcd files
DATA=`rospack find color_feature_classification`/demos

n=0
for dir in `ls $DATA/novel_view/* -d`
do
  echo $dir
  
  for((i=0;i<$1;i++))
  do
    obj=$(printf "obj%03d" $i)
    mkdir $dir/$obj 2> /dev/null
  done

  bash `rospack find color_feature_classification`/scripts/demos/2.computeFeatureAtDirectory.sh $dir

  echo hist_data_forSVM_real/novel_view/$n
  mkdir hist_data_forSVM_real/novel_view/$n -p
  cp -r $dir/test_features_c/ hist_data_forSVM_real/novel_view/$n/
  cp -r $dir/test_features_d/ hist_data_forSVM_real/novel_view/$n/
  cp -r $dir/test_features_g/ hist_data_forSVM_real/novel_view/$n/
  cp -r $dir/test_features_r/ hist_data_forSVM_real/novel_view/$n/
  ((n++))
done
