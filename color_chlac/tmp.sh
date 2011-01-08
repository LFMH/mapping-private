#!/bin/bash

for noise in noiseless noisy
do
    for shape in cone cube cylinder dice plane sphere torus
    do
	for color in black blue green orange purple red yellow
	do
	    #cp ~/tmp/shapes_noise_0001/${noise}_${shape}_${color}.pcd demos/shape_data/${noise}_${shape}_${color}.pcd
	    #rosrun color_chlac convertOld2NewPCD demos/shape_data/${noise}_${shape}_${color}.pcd demos/shape_data/${noise}_${shape}_${color}.pcd
	    #rosrun color_chlac test_GRSD_CCHLAC demos/shape_data/${noise}_${shape}_${color}.pcd

	    #git add demos/shape_data/${noise}_${shape}_${color}.pcd
	    #git add demos/shape_data/${noise}_${shape}_${color}_GRSD_CCHLAC.pcd

	    #rosrun pcl_visualization pcd_viewer demos/shape_data/${noise}_${shape}_${color}.pcd
	    #diff demos/shape_data/noiseless_${shape}_${color}.pcd ../color_feature_classification/demos_artificial/data/obj_${shape}_${color}.pcd

	    #cp demos/shape_data/noisy_${shape}_${color}.pcd ../color_feature_classification/demos_artificial/data/noisy_${shape}_${color}.pcd
	done
    done
done