#!/bin/bash

SUB_ITERS=10

echo "" > launcher_log
if [ "$1" != "" ];
then
    SUB_ITERS=$1
else
    echo $SUB_ITERS " iterations (default)"
fi

trap "exit" INT
echo "############################## lidar #############################"
./launcher.sh "sens_t:=lidar  adjust:=false  f_arch:=/home/galanton/catkin_ws/trajectory_archive_lidar"   $SUB_ITERS

echo "############################## rgbd ##############################"
./launcher.sh "sens_t:=rgbd   adjust:=false  f_arch:=/home/galanton/catkin_ws/trajectory_archive_rgbd"    $SUB_ITERS

echo "############################# camera #############################"
./launcher.sh "sens_t:=camera adjust:=false  f_arch:=/home/galanton/catkin_ws/trajectory_archive_cam_sim" $SUB_ITERS

echo "######################### camera + adjust ########################"
./launcher.sh "sens_t:=camera adjust:=true   f_arch:=/home/galanton/catkin_ws/trajectory_archive_cam_adj" $SUB_ITERS
