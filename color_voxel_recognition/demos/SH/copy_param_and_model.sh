#!/bin/bash
INDIR=`rospack find color_feature_classification`/demos
OUTDIR=`rospack find color_voxel_recognition`/demos
voxel_size=`cat $INDIR/voxel_size.txt`

##################################
# parameters

# The number of the dimension of feature space
# NOTE: not necessary for GRSD
dim_feature=50

# The size of the subdivisions = The step size for sliding box. ( $subdivision_size * $voxel_size = actual length (meter) )
subdivision_size=10 # 5 for voxel_size=0.02

##################################

mkdir -p $OUTDIR/param
cp $INDIR/color_threshold.txt $OUTDIR/param/
cp $INDIR/bin_normalization/max_r.txt $OUTDIR/param/
cp $INDIR/bin_normalization/max_c.txt $OUTDIR/param/
cp $INDIR/bin_normalization/max_g.txt $OUTDIR/param/
cp $INDIR/bin_normalization/max_d.txt $OUTDIR/param/
echo "voxel_size: $voxel_size" > $OUTDIR/param/parameters.txt
echo "dim: $dim_feature" >> $OUTDIR/param/parameters.txt
echo "box_size(scene): $subdivision_size" >> $OUTDIR/param/parameters.txt

# model
mkdir -p $OUTDIR/models_offline_r
mkdir -p $OUTDIR/models_offline_c
mkdir -p $OUTDIR/models_offline_g
mkdir -p $OUTDIR/models_offline_d
cp $INDIR/pca_result_r/compress_axis $OUTDIR/models_offline_r
cp $INDIR/pca_result_c/compress_axis $OUTDIR/models_offline_c
cp $INDIR/pca_result_g/compress_axis $OUTDIR/models_offline_g
cp $INDIR/pca_result_d/compress_axis $OUTDIR/models_offline_d
for((i=0;i<63;i++))
do
    num=$(printf "%03d" $i)
    mkdir -p $OUTDIR/models_offline_r/$num
    mkdir -p $OUTDIR/models_offline_c/$num
    mkdir -p $OUTDIR/models_offline_g/$num
    mkdir -p $OUTDIR/models_offline_d/$num
    cp $INDIR/pca_result_r/$num $OUTDIR/models_offline_r/$num/pca_result
    cp $INDIR/pca_result_c/$num $OUTDIR/models_offline_c/$num/pca_result
    cp $INDIR/pca_result_g/$num $OUTDIR/models_offline_g/$num/pca_result
    cp $INDIR/pca_result_d/$num $OUTDIR/models_offline_d/$num/pca_result
done
