#!/bin/bash
# Apache License 2.0
# Copyright (c) 2023, CuboRex Inc.

echo ""
echo "Install the sensor packages used by CuGo."
echo ""

echo "Only apt installs use sudo privileges."
sudo apt update

cd ../..

echo "Install RPLIDAR package"
git clone https://github.com/Slamtec/rplidar_ros.git
git clone https://github.com/ros-perception/laser_filters.git
cd laser_filters
git checkout kinetic-devel
cd ..
echo ""

echo "Install Wit Motion IMU package"
sudo apt install ros-noetic-ecl*
git clone https://github.com/yowlings/wit_node.git
echo ""

echo "Install EMCL2"
git clone https://github.com/ryuichiueda/emcl2.git
echo ""

echo "Install Navigation Package"
sudo apt install -y libbullet-dev libsdl-image1.2-dev libsdl-dev
sudo apt install -y ros-noetic-navigation
sudo apt install -y ros-noetic-geometry2
echo ""

echo "Instal cartgrapher ros"
sudo apt install -y python3-wstool python3-rosdep ninja-build stow
sudo apt install -y python3-osrf-pycommon python3-catkin-tools
sudo apt install -y build-essential
sudo apt install -y python3-osrf-pycommon python3-catkin-tools
sudo apt install -y google-mock
sudo apt install -y libgmock-dev
sudo apt install -y libceres-dev
cd ../
wstool init src
wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
src/cartographer/scripts/install_abseil.sh


catkin build
