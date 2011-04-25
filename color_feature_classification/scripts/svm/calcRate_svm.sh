#!/bin/bash

results=(`cat result_svm_raw.txt | grep Accuracy`)
num=${#results[@]}
rotation_flg=-1
count=0
result_line=""

touch result_svm_0.txt
touch result_svm_1.txt

for((i=0;i<$num;i++))
do
 if [ ${results[$i]} = "=" ]
 then
     i=`expr $i + 1`
     result_line=`echo $result_line ${results[$i]}`
     count=`expr $count + 1`
     if [ $count -eq 4 ]
     then
	 if [ $rotation_flg -eq -1 ]
	 then
	     rotation_flg=0
	 else
	     echo $result_line >> result_svm_$rotation_flg.txt
	     if [ $rotation_flg -eq 0 ]
	     then
		 rotation_flg=1
	     else
		 rotation_flg=0
	     fi
	 fi
	 result_line=""
	 count=0
     fi
 fi
done