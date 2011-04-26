#!/bin/bash
# Example directory containing _vfh.pcd files
DATA=`rospack find color_feature_classification`/demos

###### test 64 objects
# ln -s test_data/test_features_c .
# ln -s test_data/test_features_d .
# ln -s test_data/test_features_g .
# ln -s test_data/test_features_r .

#     # ln -s features_backup/C3_noNormalize_rotate$mode/features_c .
#     # ln -s features_backup/C3_noNormalize_rotate$mode/features_d .
#     # ln -s features_backup/C3_noNormalize_rotate$mode/features_g .
#     # ln -s features_backup/C3_noNormalize_rotate$mode/features_r .
#     # bash ../scripts/demos/0.calcNormalization.sh
#     # bash ../scripts/demos/0.compute_compress_axis.sh
#     # bash ../scripts/demos/1.compute_subspace.sh
#     # mkdir pca_backup_c3_whitening_rotate$mode
#     # cp -r pca_result_* pca_backup_c3_whitening_rotate$mode
#     # rm features_c
#     # rm features_d
#     # rm features_g
#     # rm features_r
#     ln -s pca_backup_c3_whitening_rotate$mode/pca_result_c .
#     ln -s pca_backup_c3_whitening_rotate$mode/pca_result_d .
#     ln -s pca_backup_c3_whitening_rotate$mode/pca_result_g .
#     ln -s pca_backup_c3_whitening_rotate$mode/pca_result_r .
    
    # for dim in 50 #10 20 40 70
    # do
    # 	bash ../scripts/demos/2.test_classify.sh $dim >results_dim$dim.txt
    # done
#     rm pca_result_c
#     rm pca_result_d
#     rm pca_result_g
#     rm pca_result_r

######################
###### test novel objects 
for dim in 70 #50 5 10 15 20 30 40 60 70 80 90 #200 300 400 500 600 700 800 900
do
    #touch results_icra2011setup_noNormalize_clafic_dim$dim.txt
    touch $2
    
    for dir_name in `ls $1`
    do
	echo "#####" >>$2
	echo $dir_name >>$2
	echo "#####" >>$2

    # get $num
	num1=`ls $1/$dir_name/obj* -d | wc -l`
	num2=`ls $1/$dir_name/obj*/*.pcd | wc -l`
	num=`expr $num2 / $num1`
	echo $num
	
	#bash $DATA/../scripts/demos/2.test_classify_novel.sh $num $dim $1/$dir_name >hogehoge
	bash $DATA/../scripts/demos/2.test_classify_novel.sh $num $dim $1/$dir_name >>$2
	# echo "#####" >>results_icra2011setup_noNormalize_clafic_dim$dim.txt
	# echo $dir_name >>results_icra2011setup_noNormalize_clafic_dim$dim.txt
	# echo "#####" >>results_icra2011setup_noNormalize_clafic_dim$dim.txt
	# bash ../scripts/demos/calcRate.sh hogehoge $num >>results_icra2011setup_noNormalize_clafic_dim$dim.txt
	# rm hogehoge
    done
done