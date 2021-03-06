#!/bin/bash

#@ brief: this script creates the debian package
# for the mentioned package name
#NOTE: ROS and workspace has to sourced

#author: puru rastogi puru@mowito.in


# use 'dpkg --print-architecture' or 'uname -m'to find the architechture


branch=$1
ros_version=$2

if [ "$1" = "-h" ] || [ "$2" = "-h" ] || [ "$3" = "-h" ] || [ "$4" = "-h" ];then
    echo "help:"
    exho "How to Run"
    echo "rosrun create_branch branch_name ros_version"
    echo " creates branch in all the (interally) listed repo, with name *branch_name*"
    echo "==========="
    exit 0

fi


# source the ROS
echo "============================================"
echo "Beginning the process"
echo "============================================"
source /opt/ros/${ros_version}/setup.bash
source $HOME/mowito_ws/devel/setup.bash


# first creating debians for license package
# make sure that the release branch already exist for, so that you get the key you want 


# creating debians for all the required packages except mlicense    
pkg_names=(
    mlicense
    mw_core
    mw_msgs
    mw_run
    rosbot_ekf
    mwpfl
    timed_roslaunch
    utils
    mw_mapping
    chief_executive
    costmap_2d
    diagnostic
    roboview
    executive
    task_executive
    controller_executive
    recovery_executive
    global_planner
    mw_navfn_planner
    map_exploration
)


for pkg_name in "${pkg_names[@]}"
do

    printf "\n"
    echo "============================================"
    echo "checking out ${pkg_name}"
    echo "============================================"

    #go to the pkg
    printf "\n"
    roscd ${pkg_name}
    echo "============================================"
    pwd
    echo "============================================"

    printf "\n"
    echo "============================================"
    echo "creating branch ${branch}"
    echo "============================================"
    if git checkout -b ${branch} > /dev/null 2>&1
    then 
        git push -u origin ${branch}
        echo "${branch} branch created"
    else
        echo "${branch} exists"
        git checkout ${branch}
    fi

done

printf "\n"
echo "==================================================="
echo "SUCCESS!"
echo "==================================================="
