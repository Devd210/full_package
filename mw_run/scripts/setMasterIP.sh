#!/bin/bash

#@ brief: this script sets ros master URI and ROS IP for rosbot

#author: puru rastogi puru@mowito.in

ground_station_ip=$1
if [ -z $ground_station_ip ]
then
    echo 'Using puru dell latop ip'
    ground_station_ip="rifle.local"
fi

export ROS_MASTER_URI=http://husarion.local:11311
export ROS_IP=$ground_station_ip