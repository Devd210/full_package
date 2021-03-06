#!/bin/bash

#@ brief: this script creates the debian package
# for the mentioned package name
#NOTE: ROS and workspace has to sourced

#author: puru rastogi puru@mowito.in


# use 'dpkg --print-architecture' or 'uname -m'to find the architechture

ros_version=$1


if [ "$1" = "-h" ] || [ "$2" = "-h" ] || [ "$3" = "-h" ] || [ "$4" = "-h" ];then
    echo "help:"
    echo "How to Run"
    echo "rosrun create_debian_package ros_version "
    echo "creates debian of paackages (in internal list) based on their current branch"
    echo "==========="
    exit 0

fi


# source the ROS
echo "============================================"
echo "Beginning the process"
echo "============================================"
source /opt/ros/${ros_version}/setup.bash
source $HOME/mowito_ws/devel/setup.bash

mkdir $HOME/mowito_ws/src/debians/


printf "\n"
echo "===================================================="
echo "Adding rosdep-${ros_version}.yaml as additional source for rosdep"
echo "===================================================="
touch 50-my-packages.list
echo "yaml file://$HOME/mowito_ws/src/mw_tools/rosdep-${ros_version}.yaml" >> 50-my-packages.list
sudo mv 50-my-packages.list /etc/ros/rosdep/sources.list.d/

rosdep update

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
    echo "Building ${pkg_name}"
    echo "============================================"

    #go to the pkg
    printf "\n"
    roscd ${pkg_name}
    echo "============================================"
    pwd
    echo "============================================"

    printf "\n"
    echo "============================================"
    echo "installing rosdeps"
    echo "============================================"
    printf "\n"
    rosdep install ${pkg_name}

    printf "\n"
    echo "============================================"
    echo "generating debian and obj files"
    echo "============================================"
    printf "\n"
    bloom-generate rosdebian --ros-distro ${ros_version}

    printf "\n"
    echo "============================================"
    echo "generating deb file"
    echo "============================================"
    printf "\n"
    sudo fakeroot debian/rules binary

    printf "\n"
    echo "============================================"
    echo "move the deb files inside"
    echo "============================================"
    mv ../*deb .
    sudo rm *.ddeb

    printf "\n"
    echo "============================================"
    echo "install deb file"
    echo "============================================"
    sudo dpkg -i *.deb

    printf "\n"
    echo "============================================"
    echo "update rosdep"
    echo "============================================"
    rosdep update

    printf "\n"
    echo "============================================"
    echo "cleaning build files"
    echo "============================================"
    sudo rm -rf obj-*/ debian/

    printf "\n"
    echo "============================================"
    echo "moving deb file to release repo"
    echo "============================================"
    sudo mv *.deb $HOME/mowito_ws/src/debians/

done

printf "\n"
echo "==================================================="
echo "Debians were created for all packages successfully "
echo " all created debians are in ~/mowito_ws/src/debians "
echo "==================================================="

# printf "\n"
# echo "==================================================="
# echo "Creating branch in release repo"
# echo "==================================================="

# cd $HOME/mowito/
# git checkout master
# if git checkout -b ${release_branch} > /dev/null 2>&1
# then 
#     git push -u origin ${release_branch}
#     echo "new ${release_branch} created"
# else
#     git checkout ${release_branch}
#     echo "${release_branch} exists"
# fi
# sudo rm -rf debians/
# sudo mv $HOME/mowito_ws/src/debians .
# git add --all
# git commit -am "Add deb files for release"
# git push -u origin ${release_branch}

# printf "\n"
# echo "==================================================="
# echo "SUCCESS!"
# echo "==================================================="
