#!/bin/bash

DATA=`pwd`
files=`find $DATA -type f \( -iname "$1*.pcd" \)`
radii=`echo _radii.pcd`


for j in $files
do
    newname=`basename $j .pcd`
    rosrun color_chlac test_GRSD_CCHLAC $j 
#    mv $DATA/radii.pcd $DATA/$newname$radii
done

#estimate feature extraction time
#before you have to define VERBOSE 1 in color_chlac/test/example_GRSD_CCHLAC.cpp
#Command:
#bash /home/pangerci/ros/tum-stacks/mapping-private/color_feature_classification/scripts/estimateGRSD.sh | awk -F ' ' {'print $5'} >> time.txt