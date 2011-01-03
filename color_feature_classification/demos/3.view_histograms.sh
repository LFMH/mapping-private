#!/bin/bash

# Example directory containing .pcd files
DATA=`pwd`/hist_data/obj008/1

files=`find $DATA -type f \( -iname "*.pcd" \)`
files1=`echo $files | tr " " "\n" | sort`
echo "files1: " $files1
rosrun pcl_visualization pcd_viewer $files1