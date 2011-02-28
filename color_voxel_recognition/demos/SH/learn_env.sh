#!/bin/bash

# USAGE
# ./SH/learn_env.sh

# すでに環境データがある場合は消去
if [ $(ls -d scene_for_PCA | wc -l) = 1 ]
then
    rm -i -r scene_for_PCA
fi

# 環境データを取得
rosrun color_voxel_recognition saveData scene_for_PCA /input:=/camera/depth/points2_throttle

# フォルダを準備
rm scene
ln -s scene_for_PCA scene
mkdir scene/Voxel

# 環境のボクセルデータを作る
rosrun color_voxel_recognition getVoxel_scene_each $(ls scene/Points | wc -l)

# RGB値の閾値を計算する
rosrun color_voxel_recognition calc_scene_autoThreshold $(ls scene/Points | wc -l)

# Color-CHLAC特徴圧縮用のPCAを計算する
rosrun color_voxel_recognition pca_scene_from_each_file $(ls scene/Points | wc -l)

