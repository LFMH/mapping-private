#!/bin/bash

# USAGE: bash scripts/detect_test.sh <target obj number> <similarity threshold>
#   e.g. bash scripts/detect_test.sh 5 0.95
DIR=`rospack find color_voxel_recognition`/demos_VOSCH

rank_num=10
exist_voxel_num_threshold=10
r_dim=49
pca=$(echo $DIR/models/$(printf "%03d" $1))
detection_box_size=0.20 # unit: meter

#rosrun color_voxel_recognition detectObj_VOSCH $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 /input:=/camera/depth/points2_throttle
rosrun color_voxel_recognition detectObj_VOSCH $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 /input:=/cloud_pcd
