#!/bin/bash

######################################################################
## Modify only these 5 lines
######################################################################
DATA=`rospack find color_feature_classification`/demos/data/obj008
HIST_DATA=`rospack find color_feature_classification`/demos/hist_data/obj008
HEIGHT=0
CAM="0.124713,0.422811/-0.0301714,-0.183418,0.7935/-0.00515188,-0.0965969,0.553536/-0.0144605,-0.939763,-0.341522/284,200/648,"
WIN_HEIGHT=200
NR_LINES=4


######################################################################
## main
######################################################################
files=`find $DATA -type f \( -iname "*.pcd" \)`
hist_files=`find $HIST_DATA -type f \( -iname "*.pcd" \)`
files_sorted=`echo $files | tr " " "\n" | sort`
hist_files_sorted=`echo $hist_files | tr " " "\n" | sort`
#echo "files_sorted: " $files_sorted
#echo "hist_files_sorted: " $hist_files_sorted


counter=0
for j in $hist_files_sorted
do
    counter=$(( $counter + 1))
    rosrun pcl_visualization pcd_viewer `echo $files_sorted | cut -d" " -f$counter` -fpc 1 -ps 5 -cam $CAM$HEIGHT &
    HEIGHT=$(( $HEIGHT + $WIN_HEIGHT))
    sleep 0.5
    list+=`echo " "$j` 
    rem=$[ $counter % $NR_LINES ]
    if [ $rem -eq 0 ]
    then
	echo "list: " $list
	rosrun pcl_visualization pcd_viewer $list &
	sleep 1
	my_pid=`pidof pcd_viewer`
	echo "my_pid: " $my_pid
	refresh_cache="a"
	while [ $refresh_cache != "n" -o $refresh_cache != "q" ]
	do
	    echo "Continue to next batch or quit [n|q]"
	    read refresh_cache
	    if [ "$refresh_cache" = "n" ] 
	    then
		kill $my_pid
		HEIGHT=0
		list=""
		break
		#continue
	    elif [ "$refresh_cache" = "q" ] 
	    then
		kill $my_pid
		exit
	    else
		echo "Unknown option"
	    fi
	done
	#break
    fi
done
