#!/bin/bash

flag=-1
count=0
dir_num=-1
dir_name=$(printf "obj%03d" `expr $dir_num + 1`)

text=(`cat $1`)
num=`echo ${#text[@]}`
for((i=0;i<$num;i++))
do
    if [ ${text[$i]} = $dir_name ]
    then
 	flag=1
	dir_num=`expr $dir_num + 1`
	dir_name=$(printf "obj%03d" `expr $dir_num + 1`)
	i=`expr $i + 1`

               # read 24 test samples' results
               # read 3 test samples' results
	for((j=0;j<24;j++))
	do
	    i=`expr $i + 1`
 	    result=${text[$i]}
	    if [ $result = $dir_num ]
	    then
		count=`expr $count + 1`
	    fi
	    i=`expr $i + 1`
	done
    else
	if [ $flag = 1 ]
	then
	    echo correct_num: $count
	    flag=-1
	fi
	count=0	
	echo ${text[$i]}
	dir_num=-1
	dir_name=$(printf "obj%03d" `expr $dir_num + 1`)
    fi
done
echo correct_num: $count
