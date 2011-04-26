#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`rospack find color_feature_classification`/demos

sub=$2

# NOTE: comment-out the followings if you don't use normalization
norm_flag_c="-norm $DATA/bin_normalization/max_c.txt"
norm_flag_d="-norm $DATA/bin_normalization/max_d.txt"
norm_flag_g="-norm $DATA/bin_normalization/max_g.txt"
norm_flag_r="-norm $DATA/bin_normalization/max_r.txt"

echo "----------------------------------------------------------"
echo "colorCHLAC - rotation variant - (981)"
echo "----------------------------------------------------------"
dirNum=`ls $3/test_features_c/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    if [ $sub -lt 981 ]
    then
	if [ `ls $3/test_features_c/$dir_name/ | wc -l` != 0 ]
	then
    	    for j in `find $3/test_features_c/$dir_name -type f -iname "*.pcd" | sort -d`
    	    do
    		rosrun color_feature_classification test_classify_from_file $j c s -sub $sub -dim 100 -comp $DATA/pca_result_c/compress_axis $norm_flag_c $DATA
    		#rosrun color_feature_classification test_classify_from_file $j c s -sub $sub $norm_flag_c $DATA
    	    done
	else
    	    for((j=0;j<$1;j++))
    	    do
    		echo "-1 0"
    	    done
	fi
    else
	for((j=0;j<$1;j++))
	do
    	    echo "-1 0"
	done
    fi
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation variant - (1001)"
echo "----------------------------------------------------------"
dirNum=`ls $3/test_features_d/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    if [ $sub -lt 1001 ]
    then
	if [ `ls $3/test_features_c/$dir_name/ | wc -l` != 0 ]
	then
    	    for j in `find $3/test_features_d/$dir_name -type f -iname "*.pcd" | sort -d`
    	    do
    		rosrun color_feature_classification test_classify_from_file $j d s -sub $sub -dim 100 -comp $DATA/pca_result_d/compress_axis $norm_flag_d $DATA
    		#rosrun color_feature_classification test_classify_from_file $j d s -sub $sub $norm_flag_d $DATA
    	    done
	else
    	    for((j=0;j<$1;j++))
    	    do
    		echo "-1 0"
    	    done
	fi
    else
	for((j=0;j<$1;j++))
	do
    	    echo "-1 0"
	done
    fi
done
#
echo "----------------------------------------------------------"
echo "GRSD (20)"
echo "----------------------------------------------------------"
dirNum=`ls $3/test_features_g/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    if [ $sub -lt 20 ]
    then
	if [ `ls $3/test_features_c/$dir_name/ | wc -l` != 0 ]
	then
    	    for j in `find $3/test_features_g/$dir_name -type f -iname "*.pcd" | sort -d`
    	    do
    		#rosrun color_feature_classification test_classify_from_file $j g s -sub $sub -dim 150 -comp $DATA/pca_result_g/compress_axis $norm_flag_g $DATA
    		rosrun color_feature_classification test_classify_from_file $j g s -sub $sub $norm_flag_g $DATA
    	    done
	else
    	    for((j=0;j<$1;j++))
    	    do
    		echo "-1 0"
    	    done
	fi
    else
    	for((j=0;j<$1;j++))
    	do
    	    echo "-1 0"
    	done
    fi
done
#
echo "----------------------------------------------------------"
echo "GRSD-colorCHLAC - rotation invariant - (137)"
echo "----------------------------------------------------------"
dirNum=`ls $3/test_features_r/* -d | wc -l`
for((i=0;i<$dirNum;i++))
do
    dir_name=$(printf "obj%03d" $i)
    echo $dir_name
    echo "---------------------------------------------------------"
    if [ $sub -lt 137 ]
    then
    	if [ `ls $3/test_features_c/$dir_name/ | wc -l` != 0 ]
    	then
    	    for j in `find $3/test_features_r/$dir_name -type f -iname "*.pcd" | sort -d`
    	    do
    		rosrun color_feature_classification test_classify_from_file $j r s -sub $sub -dim 100 -comp $DATA/pca_result_r/compress_axis $norm_flag_r $DATA
    	    #rosrun color_feature_classification test_classify_from_file $j r s -sub $sub $norm_flag_r $DATA
    	    done
    	else
    	    for((j=0;j<$1;j++))
    	    do
    		echo "-1 0"
    	    done
    	fi
    else
	for((j=0;j<$1;j++))
	do
    	    echo "-1 0"
	done
    fi
done