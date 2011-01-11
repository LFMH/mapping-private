#!/bin/bash

DATA=`pwd`
files=`find $DATA -type f \( -iname "*$1*.pcd" \)`
radii=`echo _radii.pcd`

for j in $files
do
    rosrun color_chlac exampleGRSD $j
    newname=`basename $j .pcd`
#    echo $newname
    mv $DATA/radii.pcd $DATA/$newname$radii
done