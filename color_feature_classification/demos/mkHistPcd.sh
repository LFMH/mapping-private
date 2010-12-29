#!/bin/bash

mkdir $1

for n in 000 001 002 003 004 005 006 007 008 009
do
    mkdir $1/obj$n
    for f in `find data/obj$n -type f \( -iname "*.pcd" ! -iname "*vfh*" ! -iname "*colorCHLAC*" \)`
    do
	rosrun color_chlac test_GRSD_CCHLAC $f
    done
    mv data/obj$n/*_GRSD_CCHLAC.pcd $1/obj$n
done
