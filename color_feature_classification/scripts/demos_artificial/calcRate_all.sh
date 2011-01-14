#!/bin/bash

# e.g. bash ../scripts/demos_artificial/calcRate_all.sh ./data_result

for nlevel in 0.0005 0.0010 0.0015 0.0020 0.0025 0.0030 0.0035 0.0040 0.0045 0.0050
do
    num1=0
    num2=0
    num3=0
    num4=0
    for x in a b c d e
    do
	results=(`bash ../scripts/demos_artificial/calcRate.sh $1/result_${nlevel}_${x}.txt`)
	num1=`expr ${results[0]} + $num1`
	num2=`expr ${results[1]} + $num2`
	num3=`expr ${results[2]} + $num3`
	num4=`expr ${results[3]} + $num4`
    done
    results=`echo "scale=4; $num3 * 100.0 / 245.0" | bc -l`
    results="$results `echo "scale=4; $num1 * 100.0 / 245.0" | bc -l`"
    results="$results `echo "scale=4; $num2 * 100.0 / 245.0" | bc -l`"
    results="$results `echo "scale=4; $num4 * 100.0 / 245.0" | bc -l`"
    echo $results
done