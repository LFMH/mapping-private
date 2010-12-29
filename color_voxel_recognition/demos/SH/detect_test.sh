#!/bin/bash

# USAGE
# e.g. ./SH/detect_test.sh phone1 20

rank_num=10
exist_voxel_num_threshold=10
r_dim=40
pca=$(echo models/$1/pca_result)
size=(`cat models/$1/size.txt`)

rosrun color_voxel_recognition detectObj $rank_num $exist_voxel_num_threshold $pca $r_dim ${size[0]} ${size[0]} ${size[0]} $2 /input:=/camera/depth/points2_throttle
#rosrun color_voxel_recognition detectObj $rank_num $exist_voxel_num_threshold $pca $r_dim ${size[0]} ${size[0]} ${size[0]} $2 /input:=/cloud_pcd
