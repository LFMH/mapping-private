#!/bin/bash

# USAGE
#  bash test_all.sh result.txt
#  bash calcRate_novel.sh result.txt >result_all.txt
#  bash calcRate_percentage.sh 1512 `cat result_all.txt`

nums=(`echo $@`)
results=""
for((i=1;i<$#;i++))
do
    result=`echo "scale=7; ${nums[$i]} / ${nums[0]}" | bc`
    results=`echo $results $result`
done
echo $results