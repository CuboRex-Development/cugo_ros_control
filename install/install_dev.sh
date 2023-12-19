#!/bin/bash
# Apache License 2.0
# Copyright (c) 2023, CuboRex Inc.

echo ""
echo "Install the sensor packages used by CuGo."
echo ""

sudo apt update

cd ../..


echo "Install RPLIDAR package"
sudo apt install -y ros-$ROS_DISTRO-filters
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
git clone -b ros2 https://github.com/ros-perception/laser_filters.git
echo ""


echo "--------- cmake version ---------"
cmake --version
read -p "IMUのパッケージ(witmotion_IMU_ros)は、cmake のバージョンが3.19以上でないとbuildできません。ダウンロードしますか?(y/N): " yn

case "$yn" in
	[yY]*)
		sudo apt install -y libqt5serialport5-dev
		git clone -b foxy-devel --recursive https://github.com/p3pperPi/witmotion_IMU_ros.git
		echo ""
		;;

	*)
		echo "witmotion_IMU_ros パッケージのダウンロードをスキップしました。"
		;;
esac



# echo "Install EMCL2"
# echo ""


echo "u-blox gnss package"
sudo apt install -y ros-$ROS_DISTRO-mapviz ros-$ROS_DISTRO-mapviz-plugins ros-$ROS_DISTRO-tile-map ros-$ROS_DISTRO-multires-image
git clone -b foxy-devel https://github.com/KumarRobotics/ublox.git

echo "u-blox status monitor package"
git clone https://github.com/p3pperPi/ublox_status_monitor.git


echo "Robot Localization Package"
sudo apt install -y ros-foxy-robot-localization


# echo "Install Navigation Package"
# echo ""


# echo "Instal cartgrapher ros"
# echo ""


cd ../
rosdep install -r -y -i --from-paths src

colcon build --symlink-install
