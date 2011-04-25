#!/bin/bash

# USAGE: bash scripts/detect_test.sh <numbers of the target objects>
#   e.g. bash scripts/detect_test.sh 19 1 35 23

##################################
# parameters

demos_path=`rospack find color_voxel_recognition_2`/demos

similarity_th=0

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

display_config_file=`rospack find color_voxel_recognition`/demos/display_config

##################################

rm $demos_path/models
ln -s $demos_path/models_offline_r $demos_path/models
pca=$demos_path/models/$(printf "%03d" $1)/pca_result

rm $demos_path/pca_file_names.txt 2>/dev/null
echo $pca >$demos_path/pca_file_names.txt
model_num=$#
for((i=1;i<$model_num;i++))
do
    shift 1
    echo $demos_path/models/$(printf "%03d" $1)/pca_result >>$demos_path/pca_file_names.txt
done

#rosrun color_voxel_recognition_2 detect_vosch_multi `rospack find color_voxel_recognition_2`/demos $rank_num $exist_voxel_num_threshold $demos_path/pca_file_names.txt $r_dim $detection_box_size1 $detection_box_size2 $detection_box_size3 0 $distance_th $model_num /input:=/camera/rgb/points
roslaunch color_voxel_recognition_2 detect_multi.launch /demos_path:=$demos_path /rank_num:=$rank_num /exist_voxel_num_threshold:=$exist_voxel_num_threshold /pca:=$demos_path/pca_file_names.txt /r_dim:=$r_dim /size1:=$detection_box_size1 /size2:=$detection_box_size2 /size3:=$detection_box_size3 /similarity_th:=$similarity_th /distance_th:=$distance_th /model_num:=$model_num /input:=/camera/rgb/points /display_config:=$display_config_file

#rosrun color_voxel_recognition detectObj $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size1 $detection_box_size2 $detection_box_size3 $2 $distance_th /input:=/camera/depth/points2_throttle
#rosrun color_voxel_recognition detectObj_GRSD $rank_num $exist_voxel_num_threshold $pca $r_dim $detection_box_size $detection_box_size $detection_box_size $2 $distance_th /input:=/camera/depth/points2_throttle
