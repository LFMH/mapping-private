#!/bin/bash

# USAGE: bash scripts/detect_test.sh <target obj number> <similarity threshold>
#   e.g. bash scripts/detect_test.sh 19 0.3
DIR=`pwd` #`rospack find color_voxel_recognition`/demos

##################################
# parameters

# Maximum number of the detection results
rank_num=1 #5

# If the number of occupied voxels in a detection area is less than this value, the system will skip this area.
exist_voxel_num_threshold=500 #50

# The number of the dimension of the objects's subspace
r_dim=70 #15 

# The size of detection box
# detection_box_size1=0.07 # unit: meter
# detection_box_size2=0.20 # unit: meter
# detection_box_size3=0.07 # unit: meter
detection_box_size1=0.20 # unit: meter
detection_box_size2=0.20 # unit: meter
detection_box_size3=0.20 # unit: meter

# The distance threshold of target points in scene.
distance_th=1.3 #1.1 # unit: meter

##################################

rm models
ln -s $DIR/models_offline_r $DIR/models
pca=$(echo $DIR/models/$(printf "%03d" $1)/pca_result)

rm pca_file_names.txt 2>/dev/null
echo $pca >pca_file_names.txt
model_num=$#
for((i=1;i<$model_num;i++))
do
    shift 1
    echo $DIR/models/$(printf "%03d" $1)/pca_result >>pca_file_names.txt
done

#rosrun color_voxel_recognition detectObj $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size1 $detection_box_size2 $detection_box_size3 $2 $distance_th /input:=/camera/depth/points2_throttle
rosrun color_voxel_recognition detectObj_multi $rank_num $exist_voxel_num_threshold pca_file_names.txt $r_dim $detection_box_size1 $detection_box_size2 $detection_box_size3 0 $distance_th $model_num /input:=/camera/rgb/points_throttle

#rosrun color_voxel_recognition detectObj_GRSD $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 $distance_th /input:=/camera/depth/points2_throttle
