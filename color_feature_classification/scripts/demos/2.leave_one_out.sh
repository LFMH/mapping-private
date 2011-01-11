#!/bin/bash

echo "----------------------------------------------------------"
echo "colorCHLAC - rotateion variant - (981)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find features_c/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`)
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
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 100 -comp pca_result_c/compress_axis pca_result_c/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} c s -sub 5 -dim 100 -comp pca_result_c/compress_axis
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_d/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find features_d/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`)
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
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 100 -comp pca_result_d/compress_axis pca_result_d/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} d s -sub 5 -dim 100 -comp pca_result_d/compress_axis
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_g/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find features_g/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`)
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
	rosrun color_feature_classification computeSubspace_from_file $train_models pca_result_g/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} g s -sub 5
    done
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
dirNum=`ls test_features_r/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    num=$(printf "%03d" $i)
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"

    files1=(`find features_r/$dir_name -type f -iname "*.pcd" | sort -d`)
    files2=(`find test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`)
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
	rosrun color_feature_classification computeSubspace_from_file $train_models -dim 80 -comp pca_result_r/compress_axis pca_result_r/$num 1>/dev/null
	rosrun color_feature_classification test_classify_from_file ${files2[$j]} r s -sub 5 -dim 80 -comp pca_result_r/compress_axis
    done
done
