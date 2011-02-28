#!/bin/bash
DATA=`rospack find color_feature_classification`/demos

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

echo "----------------------------------------------------------"
echo "colorCHLAC - rotation variant - (981)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find $DATA/features_c/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find $DATA/test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`)
    fileNum=`echo ${#files1[@]}`

    for((j=0; j<$fileNum; j++))
    do
	train_models=""
	#echo "Test ${files2[$j]}"
	for((k=0; k<$j; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	for((k=$j+1; k<$fileNum; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	cp -f $DATA/pca_result_c/$num hogehoge
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 50 -comp $DATA/pca_result_c/compress_axis $norm_flag_c $DATA/pca_result_c/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} c s -sub 15 -dim 50 -comp $DATA/pca_result_c/compress_axis $norm_flag_c 
	mv -f hogehoge $DATA/pca_result_c/$num
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_d/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find $DATA/features_d/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find $DATA/test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`)
    fileNum=`echo ${#files1[@]}`

    for((j=0; j<$fileNum; j++))
    do
	train_models=""
	#echo "Test ${files2[$j]}"
	for((k=0; k<$j; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	for((k=$j+1; k<$fileNum; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	cp -f $DATA/pca_result_d/$num hogehoge
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 50 -comp $DATA/pca_result_d/compress_axis $norm_flag_d $DATA/pca_result_d/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} d s -sub 15 -dim 50 -comp $DATA/pca_result_d/compress_axis $norm_flag_d
	mv -f hogehoge $DATA/pca_result_d/$num
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_g/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find $DATA/features_g/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find $DATA/test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`)
    fileNum=`echo ${#files1[@]}`

    for((j=0; j<$fileNum; j++))
    do
	train_models=""
	#echo "Test ${files2[$j]}"
	for((k=0; k<$j; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	for((k=$j+1; k<$fileNum; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	cp -f $DATA/pca_result_g/$num hogehoge
	rosrun color_feature_classification computeSubspace_from_file $train_models $norm_flag_g $DATA/pca_result_g/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} g s -sub 15 $norm_flag_g
	mv -f hogehoge $DATA/pca_result_g/$num
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
dirNum=`ls $DATA/test_features_r/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find $DATA/features_r/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find $DATA/test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`)
    fileNum=`echo ${#files1[@]}`

    for((j=0; j<$fileNum; j++))
    do
	train_models=""
	#echo "Test ${files2[$j]}"
	for((k=0; k<$j; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	for((k=$j+1; k<$fileNum; k++))
	do
	    train_models="$train_models ${files1[$k]}"
	done
	cp -f $DATA/pca_result_r/$num hogehoge
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 50 -comp $DATA/pca_result_r/compress_axis $norm_flag_r $DATA/pca_result_r/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} r s -sub 15 -dim 50 -comp $DATA/pca_result_r/compress_axis $norm_flag_r
	mv -f hogehoge $DATA/pca_result_r/$num
    done
done
