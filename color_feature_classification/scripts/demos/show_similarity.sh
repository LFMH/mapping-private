#!/bin/bash
DATA=`rospack find color_feature_classification`/demos

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

ln -s $DATA/test_data/test_features_c $DATA/test_features_c
ln -s $DATA/test_data/test_features_d $DATA/test_features_d
ln -s $DATA/test_data/test_features_g $DATA/test_features_g
ln -s $DATA/test_data/test_features_r $DATA/test_features_r

#output_dir=results_similarity_c3_noNormalize_rotate1_7_2_whitening_clafic
output_dir=results_similarity_c3_noNormalize_rotate1_7_2_whitening_multiple
#output_dir=results_similarity_c3_noNormalize_rotate1_7_2_noWhitening_clafic
#output_dir=results_similarity_c3_noNormalize_rotate1_7_2_noWhitening_multiple
mkdir -p $output_dir
mkdir -p $output_dir/colorCHLAC
mkdir -p $output_dir/VOSCH

for dim in 30 # 50 80 20 70 # 5 15 
do
    dirNum=`ls $DATA/test_features_c/* -d | wc -l`
    for((i=0;i<$dirNum;i++))
    do
	dir_name=$(printf "obj%03d" $i)
	echo $dir_name
	n=0
	for j in `find $DATA/test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`
	do
	    echo $n
	    file_name=$(printf "%03d" $n)
	    rosrun color_feature_classification test_classify_from_file $j c s -sub $dim -dim 100 -comp $DATA/pca_result_c/compress_axis $norm_flag_c >${output_dir}/colorCHLAC/${dir_name}_${file_name}.txt $DATA
	    n=`expr $n + 1`
	done
    done
###
    dirNum=`ls $DATA/test_features_r/* -d | wc -l`
    for((i=0;i<$dirNum;i++))
    do
	dir_name=$(printf "obj%03d" $i)
	echo $dir_name
	n=0
	for j in `find $DATA/test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`
	do
	    echo $n
	    file_name=$(printf "%03d" $n)
	    rosrun color_feature_classification test_classify_from_file $j r s -sub $dim -dim 100 -comp $DATA/pca_result_r/compress_axis $norm_flag_r >${output_dir}/VOSCH/${dir_name}_${file_name}.txt $DATA
	    n=`expr $n + 1`
	done
    done
done
rm $DATA/test_features_c
rm $DATA/test_features_d
rm $DATA/test_features_g
rm $DATA/test_features_r
