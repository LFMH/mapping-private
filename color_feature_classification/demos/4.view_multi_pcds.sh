#!/bin/bash

# Example directory containing .pcd files
DATA=`pwd`/data/obj008/1

files=`find $DATA -type f \( -iname "*.pcd" \)`
files1=`echo $files | tr " " "\n" | sort`
echo "files1: " $files1
rosrun pcl_visualization pcd_viewer $files1 -multiview 1 -fpc 1 -ps 5 -ps 5 -ps 5 -ps 5 -ps 5 -ps 5 -cam 0.174981,0.483448/-0.0265333,-0.182824,0.7945/0.0167812,-0.120643,0.497866/-0.0353321,-0.977068,-0.209975/1432,822/8,73