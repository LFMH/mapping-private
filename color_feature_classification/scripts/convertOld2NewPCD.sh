#!/bin/bash

DATA=`rospack find color_feature_classification`/demos_artificial/data

for noise in noiseless
do
    for shape in cone cube cylinder dice plane sphere torus
    do
	for color in black blue green orange purple red yellow
	do
	    rosrun color_feature_classification convertOld2NewPCD $DATA/${noise}_${shape}_${color}.pcd $DATA/${noise}_${shape}_${color}.pcd
	done
    done
done