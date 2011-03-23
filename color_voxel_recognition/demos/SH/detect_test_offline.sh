#!/bin/bash

# USAGE: bash scripts/detect_test.sh <target obj number> <similarity threshold>
#   e.g. bash scripts/detect_test.sh 19 0.3
DIR=`pwd` #`rospack find color_voxel_recognition`/demos

##################################
# parameters

# Maximum number of the detection results
rank_num=1 #5

# If the number of occupied voxels in a detection area is less than this value, the system will skip this area.
exist_voxel_num_threshold=200 #50

# The number of the dimension of the objects's subspace
r_dim=70 #15 

# The size of detection box
detection_box_size=0.20 # unit: meter

# The distance threshold of target points in scene.
distance_th=1.1 # unit: meter

##################################

rm models
ln -s $DIR/models_offline_r $DIR/models
pca=$(echo $DIR/models/$(printf "%03d" $1)/pca_result)
rosrun color_voxel_recognition detectObj $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 $distance_th /input:=/camera/depth/points2_throttle
#rosrun color_voxel_recognition detectObj_GRSD $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 $distance_th /input:=/camera/depth/points2_throttle
