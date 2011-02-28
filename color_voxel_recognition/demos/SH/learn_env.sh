#!/bin/bash

# USAGE
# ./SH/learn_env.sh

# ���Ǥ˴Ķ��ǡ�����������Ͼõ�
if [ $(ls -d scene_for_PCA | wc -l) = 1 ]
then
    rm -i -r scene_for_PCA
fi

# �Ķ��ǡ��������
rosrun color_voxel_recognition saveData scene_for_PCA /input:=/camera/depth/points2_throttle

# �ե���������
rm scene
ln -s scene_for_PCA scene
mkdir scene/Voxel

# �Ķ��Υܥ�����ǡ�������
rosrun color_voxel_recognition getVoxel_scene_each $(ls scene/Points | wc -l)

# RGB�ͤ����ͤ�׻�����
rosrun color_voxel_recognition calc_scene_autoThreshold $(ls scene/Points | wc -l)

# Color-CHLAC��ħ�����Ѥ�PCA��׻�����
rosrun color_voxel_recognition pca_scene_from_each_file $(ls scene/Points | wc -l)

