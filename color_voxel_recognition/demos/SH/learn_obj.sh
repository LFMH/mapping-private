#!/bin/bash

# USAGE
# e.g. ./SH/learn_obj.sh phone1

mkdir models_online
rm models
ln -s models_online models
cp scene/pca_result models/compress_axis

# すでに物体データがある場合は消去
if [ $(ls -d models/$1 | wc -l) = 1 ]
then
    rm -i -r models/$1
fi

# 検出対象物体データを取得
rosrun color_voxel_recognition saveData models/$1 0.1 /input:=/camera/depth/points2_throttle

# フォルダを準備
mkdir models/$1/Voxel

# 物体のボクセルデータを作る（回転させT姿勢分を作成）
rosrun color_voxel_recognition getVoxel_model $1 $(ls models/$1/Points | wc -l)

# 物体毎の部分空間の基底を求める（類似度計算に使用）
rosrun color_voxel_recognition pca_models $1 $(ls models/$1/Voxel | wc -l)
