#!bin/bash
DATA=`rospack find color_feature_classification`/demos

##########
# prepare
# for dirname in arbitrary_rotation different_light free_view occlusion1 occlusion2 similar_shape textured textureless
# do
#     mkdir ~/tmp/novel_view_hist/$dirname
#     for x in c d g r
#     do
# 	mkdir ~/tmp/novel_view_hist/$dirname/test_features_$x
# 	for((n=0;n<63;n++))
# 	do
# 	    num=$(printf "%03d" $n)
# 	    mkdir ~/tmp/novel_view_hist/$dirname/test_features_$x/obj$num
# 	done

#                # computeFeature
# 	obj_dirs=(`ls ~/tmp/novel_view/$dirname`)
# 	obj_dirs_num=`echo ${#obj_dirs[@]}`
# 	for((n=0;n<$obj_dirs_num;n++))
# 	do
# 	    obj_files=(`ls ~/tmp/novel_view/$dirname/${obj_dirs[$n]}`)
# 	    obj_files_num=`echo ${#obj_files[@]}`
# 	    for((m=0;m<$obj_files_num;m++))
# 	    do
# 		num=$(printf "%03d" $m)
# 		rosrun color_feature_classification computeFeature ~/tmp/novel_view/$dirname/${obj_dirs[$n]}/${obj_files[$m]} $x $DATA ~/tmp/novel_view_hist/$dirname/test_features_$x/${obj_dirs[$n]}/$num.pcd
# 	    done
# 	done
#     done
# done

########
# test
for dirname in arbitrary_rotation different_light free_view occlusion1 occlusion2 similar_shape textured textureless
do
    for x in c d g r
    do
	rm $DATA/test_features_$x
	ln -s ~/tmp/novel_view_hist/$dirname/test_features_$x $DATA/test_features_$x
    done
    obj_dirs=(`ls ~/tmp/novel_view/$dirname`)
    sample_num=$(ls ~/tmp/novel_view/$dirname/${obj_dirs[0]} | wc -l)
    bash $DATA/../scripts/demos/2.test_classify_novel.sh $sample_num > $DATA/data_result_novel/$dirname.txt
done
