#!/bin/bash
INDIR=`rospack find color_feature_classification`/demos
OUTDIR=`rospack find color_voxel_recognition`/demos_VOSCH

# parameters
voxel_size=`cat $INDIR/voxel_size.txt`
dim_feature=50
subdivision_size=10

mkdir $OUTDIR/param
cp $INDIR/color_threshold.txt $OUTDIR/param/
cp $INDIR/bin_normalization/minmax_r.txt $OUTDIR/param/
echo "voxel_size: $voxel_size" > $OUTDIR/param/parameters.txt
echo "dim: $dim_feature" >> $OUTDIR/param/parameters.txt
echo "box_size(scene): $subdivision_size" >> $OUTDIR/param/parameters.txt

# model
mkdir $OUTDIR/models
cp $INDIR/pca_result_r/compress_axis $OUTDIR/models/
for((i=0;i<63;i++))
do
    num=$(printf "%03d" $i)
    cp $INDIR/pca_result_r/$num $OUTDIR/models/
done
