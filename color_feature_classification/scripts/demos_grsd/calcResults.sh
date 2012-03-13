#!/bin/bash

DIR=`rospack find color_feature_classification`/demos_grsd
mkdir -p $DIR/results_g
mkdir -p $DIR/results_c
mkdir -p $DIR/results_r

rm $DIR/results_g/result_all.txt 2>/dev/null
touch $DIR/results_g/result_all.txt
for sub in 3 5 10 15 18
do
    bash `rospack find color_feature_classification`/scripts/demos_grsd/2.test_classify.sh g $sub > $DIR/results_g/result_$sub.txt
    bash `rospack find color_feature_classification`/scripts/demos_grsd/calcRate.sh $DIR/results_g/result_$sub.txt $sub >>$DIR/results_g/result_all.txt
done

rm $DIR/results_c/result_all.txt 2>/dev/null
touch $DIR/results_c/result_all.txt
for sub in 3 5 10 15 18 20 30 40 50 60 70 80 90 100
do
    bash `rospack find color_feature_classification`/scripts/demos_grsd/2.test_classify.sh c $sub > $DIR/results_c/result_$sub.txt
    bash `rospack find color_feature_classification`/scripts/demos_grsd/calcRate.sh $DIR/results_c/result_$sub.txt $sub >>$DIR/results_c/result_all.txt
done

rm $DIR/results_r/result_all.txt 2>/dev/null
touch $DIR/results_r/result_all.txt
for sub in 3 5 10 15 18 20 30 40 50 60 70 80 90 100
do
    bash `rospack find color_feature_classification`/scripts/demos_grsd/2.test_classify.sh r $sub > $DIR/results_r/result_$sub.txt
    bash `rospack find color_feature_classification`/scripts/demos_grsd/calcRate.sh $DIR/results_r/result_$sub.txt $sub >>$DIR/results_r/result_all.txt
done


