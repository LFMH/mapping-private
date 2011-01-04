#!/bin/bash

# Example directory containing .pcd files
DATA=`pwd`/data/obj008/1
HIST_DATA=`pwd`/hist_data/obj008/1

files=`find $DATA -type f \( -iname "*.pcd" \)`
hist_files=`find $HIST_DATA -type f \( -iname "*.pcd" \)`
files_sorted=`echo $files | tr " " "\n" | sort`
hist_files_sorted=`echo $hist_files | tr " " "\n" | sort`
echo "files_sorted: " $files_sorted
rosrun pcl_visualization pcd_viewer $hist_files_sorted &
HEIGHT=0
WIN_HEIGHT=200
for j in $files_sorted
do
#    echo "j: " $j
    rosrun pcl_visualization pcd_viewer $j -fpc 1 -ps 5 -cam 0.124713,0.422811/-0.0301714,-0.183418,0.7935/-0.00515188,-0.0965969,0.553536/-0.0144605,-0.939763,-0.341522/284,200/648,$HEIGHT &
    HEIGHT=$(( $HEIGHT + $WIN_HEIGHT))
    echo "HEIGHT: " $HEIGHT
    sleep 0.2
done
