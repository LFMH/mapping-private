#!/bin/bash

# Example directory containing _colorCHLAC.pcd files
DATA=`pwd`/data

for i in `find $DATA -type d -name "*"`
do
  echo $i
  for j in `find $i -type f \( -iname "*_colorCHLAC.pcd" \) | sort -n`
  do
    all_files=`echo $all_files" "$j`
    break
  done
done

rosrun pcl_visualization pcd_viewer $all_files
