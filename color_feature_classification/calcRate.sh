#!/bin/bash

flag=-1
count=0

text=(`cat $1`)
num=`echo ${#text[@]}`
for((i=0;i<$num;i++))
do
    if [ ${text[$i]} = 0 ]
    then
 	flag=1
    fi
    if [ $flag = 1 ]
    then
 	ans=${text[$i]}
	i=`expr $i + 1`
 	result=${text[$i]}
	if [ ${text[`expr $i - 1`]} = ${text[$i]} ]
	then
	    count=`expr $count + 1`
	fi
	i=`expr $i + 1`
	if [ ${text[`expr $i - 2`]} = 48 ]
	then
 	    flag=-1
	    echo $count
	    count=0
	fi
    else
	echo ${text[$i]}
    fi
done

# for i in `cat $1`
# do
#     echo $i
#     if [ $i = 0 ]
#     then
# 	num=0
#     fi
#     if [ $num != -1 ]
#     then
# 	num=$i
#     fi
# done