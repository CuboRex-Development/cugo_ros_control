# cugo-ros-controller

CuGoをROSで制御する際、ROS開発キットに付属するArduinoに対して制御指令を送り、エンコーダの読み取り結果を受け取るノードです。ROSTopicの/cmd_velをSubscribeし、/odomをPublishします。セットでArduinoドライバと同時に使用します。
Arduinoドライバのリポジトリはこちら： https://github.com/CuboRex-Development/cugo-ros-arduinodriver.git
English Documents here：
正式リリースするまでは、beta branchで管理しますので、そちらをご参照ください。

# Table of Contents
- [Features](#features)
- [Requirement](#requirement)
- [Installation](#installation)
- [Usage](#usage)
- [Topics and Parameters](#topics-and-parameters)
- [Note](#note)
- [License](#license)

# Features
CuGo-ROS-ArduinoDriverに対して、/cmd_velのベクトルを達成するためのロボットのL/Rの回転数を計算し指示を投げます。また、Arduinoからエンコーダのカウント数を受け取ることでロボットのオドメトリ座標を計算し、/odomを生成しPublishします。
Arduinoとの通信はUDP通信にて実現します。ロボット内のEdgeルータに対して、有線Ethernetケーブルまたは、WiFiに接続することでArduinoと通信することができます。Arduinoドライバで受付IPを指定し、そのIPに対してUDP信号を投げます。デフォルトでは、192.168.8.216に対して投げます。必要に応じて変更してください。

# Requirement
- OS: Ubuntu 20.04.4 LTS
- ROS Distribution: Noetic Ninjemys

# Installation
ROS環境がない場合は[ROS Wiki](http://wiki.ros.org/ja/noetic/Installation/Ubuntu)を参照しROSをインストールしてください。

ROSのワークスペース内でgit cloneしたのち、catkin buildしてください。
~~~
$ cd ~/your/ros_workspace/catkin_ws/src
$ git clone https://github.com/CuboRex-Development/cugo-ros-controller.git 
$ cd ../..
$ catkin build
$ source ~/your/ros_workspace/catkin_ws/devel/setup.bash
~~~

# Usage

## 実行方法
~~~
$ roslaunch CuGoPy_Controller start_cugo_controller.launch
~~~
→ノード名が旧名なので、"cugo-ros-controller"に後ほど変更します。

以下のLaunchで起動すると、joy_nodeやteleop_joy_nodeを同時に起動することができます。
~~~
$ roslaunch CuGoPy_Controller start_ps4_controller.launch
~~~
他に、任意のノードと同時に起動したい場合、こちらのlaunchファイルを編集して使用してください。
~~~
$ roslaunch CuGoPy_Controller start_any_apps.launch
~~~

# Topics and Parameters
## Published Topics
- /cugo_controller/odom ([nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html))
- /tf ([tf2_msgs/TFMessage](http://docs.ros.org/en/jade/api/tf2_msgs/html/msg/TFMessage.html))

## Subscribed Topics
- /cmd_vel ([geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html))

## Parameters
- ~arduino_addr (string, default: 192.168.8.216)
  - Arduinoドライバの通信受付IPアドレス
- ~arduino_port (int, default: 8888)
  - Arduinoドライバの通信受付ポート番号
- ~encoder_max (int, default: 2147483647)
  - エンコーダ最大カウント
- ~encoder_resolution (int, default: 2048)
  - エンコーダ分解能
- ~odom_child_frame_id (string, default: base_link)
- ~odom_frame_id (string, default: odom)
- ~reduction_ratio (float, default: 1.0)
  - 減速比
- ~timeout (float, default: 0.05)
  - 通信タイムアウトまでの時間[sec]
- ~tread (float, default: 0.380)
  - トレッド幅[m]
- ~wheel_radius_l (float, default: 0.03858)
  - 左タイヤ半径[m]
- ~wheel_radius_r (float, default: 0.03858)
  - 右タイヤ半径[m]

上記のパラメータはlaunchファイルで設定されています。

# Note

クローラ走行の振動が非常に大きいので、RJ45端子のEthernetケーブルでの通信 / WiFi接続による通信をお勧めします。　　
シリアル通信ものちに対応予定です。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
