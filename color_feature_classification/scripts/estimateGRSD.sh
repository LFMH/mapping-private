#!/bin/bash

DATA=`pwd`
files=`find $DATA -type f \( -iname "*$1*.pcd" \)`
radii=`echo _radii.pcd`

for j in $files
do
    newname=`basename $j .pcd`
    echo "Object: " $newname
    rosrun color_chlac exampleGRSD $j 
    mv $DATA/radii.pcd $DATA/$newname$radii
done