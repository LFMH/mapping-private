#!/bin/bash

# USAGE
# e.g. ./SH/learn_obj.sh phone1

mkdir models_online
rm models
ln -s models_online models
cp scene/pca_result models/compress_axis

# ���Ǥ�ʪ�Υǡ�����������Ͼõ�
if [ $(ls -d models/$1 | wc -l) = 1 ]
then
    rm -i -r models/$1
fi

# �����о�ʪ�Υǡ��������
rosrun color_voxel_recognition saveData models/$1 0.1 /input:=/camera/depth/points2_throttle

# �ե���������
mkdir models/$1/Voxel

# ʪ�ΤΥܥ�����ǡ�������ʲ�ž����T����ʬ�������
rosrun color_voxel_recognition getVoxel_model $1 $(ls models/$1/Points | wc -l)

# ʪ�������ʬ���֤δ������������ٷ׻��˻��ѡ�
rosrun color_voxel_recognition pca_models $1 $(ls models/$1/Voxel | wc -l)
