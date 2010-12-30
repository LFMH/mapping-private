#!/bin/bash

# Example directory containing .pcd files
DATA=`pwd`/data/obj008/1

#for i in `find $DATA -type d -name "*"`
#do
#  echo $i
# for j in `find $DATA -type f \( -iname "*.pcd" \)`
# do
#     echo "j: " $j
#     all_files=`echo $all_files" "$j`
#     break
# done
#done

# echo "all_files: " $all_files
files=`find $DATA -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
rosrun pcl_visualization pcd_viewer $files