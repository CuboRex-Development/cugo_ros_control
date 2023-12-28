	#!/bin/bash
# Apache License 2.0
# Copyright (c) 2023, CuboRex Inc.

echo ""
echo "Install the sensor packages used by CuGo."
echo ""

source /opt/ros/$ROS_DISTRO/setup.bash

echo "--------- cmake version ---------"

cmake --version

read -p "cmake のバージョンは3.19以上でないとbuildできません。続けますか?(y/N): " yn
case "$yn" in [yY]*) ;; *) echo "abort." ; exit ;; esac

sudo apt update

cd ../..


echo "Install RPLIDAR package"
sudo apt install -y ros-$ROS_DISTRO-filters
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
git clone -b ros2 https://github.com/ros-perception/laser_filters.git
echo ""


# ビルドのためには、ver3.19以上のCMakeが必要
echo "Install Wit Motion IMU package"
sudo apt install -y libqt5serialport5-dev
git clone -b --recursive https://github.com/p3pperPi/witmotion_IMU_ros.git
echo ""


# echo "Install EMCL2"
# echo ""


#echo "u-blox gnss package"
#sudo apt install -y ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-multires-image
#git clone -b foxy-devel https://github.com/KumarRobotics/ublox.git

#echo "u-blox status monitor package"
#git clone https://github.com/p3pperPi/ublox_status_monitor.git


#echo "Robot Localization Package"
#sudo apt install -y ros-foxy-robot-localization
#git clone -b foxy-devel https://github.com/nobleo/robot_localization.git


echo "Install Navigation Package"
echo ""
sudo apt install \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-turtlebot3*


echo "Instal cartgrapher ros"
echo ""
sudo apt install ros-$ROS_DISTRO-cartographer

cd ..

rosdep install -r -y -i --from-paths src

colcon build --symlink-install
