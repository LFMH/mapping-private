#!/bin/bash

# USAGE
#  bash test_all.sh result.txt
#  bash calcRate_novel.sh result.txt

text=(`cat $1`)
num=`echo ${#text[@]}`

count=0
flg=0
result=""
result1=0
result2=0
result3=0
result4=0
for((i=0;i<$num;i++))
do
    #echo ${text[$i]}
    if [ $flg = 1 ]
    then
	count=`expr $count + 1`
	result=`echo $result ${text[$i]}`
	if [ $count = 1 ]
	then
	    result1=`expr $result1 + ${text[$i]}`
	fi
	if [ $count = 2 ]
	then
	    result2=`expr $result2 + ${text[$i]}`
	fi
	if [ $count = 3 ]
	then
	    result3=`expr $result3 + ${text[$i]}`
	fi
	if [ $count = 4 ]
	then
	    result4=`expr $result4 + ${text[$i]}`
	fi
	flg=0
    fi
    if [ $count = 4 ]
    then
	echo $result
	result=""
	count=0
    fi
    if [ ${text[$i]} = "correct_num:" ]
    then
	flg=1
    fi
done

echo "--------"
echo $result1 $result2 $result3 $result4