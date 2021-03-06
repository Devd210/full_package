#!/usr/bin/env bash
ros_version=$1

sudo apt install ros-$ros_version-tf2-geometry-msgs
sudo apt install ros-$ros_version-diagnostic-updater
sudo apt install ros-$ros_version-cmake-modules
sudo apt install ros-$ros_version-tf2-sensor-msgs
sudo apt install ros-$ros_version-voxel-grid
sudo apt install ros-$ros_version-sbpl
