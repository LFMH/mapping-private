#!/bin/bash

flag=-1
true_count=0
false_count=0
#dir_num=0 #-1

text=(`cat $1`)
num=`echo ${#text[@]}`
for((i=0;i<$num;i++))
do
    if [ `echo ${text[$i]} | cut -c1-3` = "obj" ]
    then
	#echo === ${text[$i]} ===
 	flag=1
	dir_num=`echo ${text[$i]} | cut -c6-6`
    else
	if [ $flag = 1 ]
	then
	    flag=-1
	else
 	    result=${text[$i]}
	    if [ $result = $dir_num ]
	    then
		true_count=`expr $true_count + 1`
	    else
		false_count=`expr $false_count + 1`
	    fi
	    i=`expr $i + 1`
	fi
    fi
done
#echo \(true false\) : \($true_count $false_count\)
sample_num=`expr $true_count + $false_count`
echo $2 `echo "scale=7; $true_count/$sample_num" | bc`
