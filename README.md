# cugo-ros-controller

CuGoをROSで制御する際、ROS開発キットに付属するArduinoに対して制御指令を送り、エンコーダの読み取り結果を受け取るノードです。ROSTopicの/cmd_velをSubscribeし、/odomをPublishします。セットでArduinoドライバと同時に使用します。

Arduinoドライバのリポジトリはこちら： https://github.com/CuboRex-Development/cugo-ros-arduinodriver.git

# Table of Contents
- [Features](#features)
- [Requirement](#requirement)
- [Installation](#installation)
- [Usage](#usage)
- [Topics and Parameters](#topics-and-parameters)
- [UDP Protocol](#udp-protocol)
- [Note](#note)
- [License](#license)

# Features
CuGo-ROS-ArduinoDriverに対して、/cmd_velのベクトルを達成するためのロボットのL/Rの回転数を計算し指示を投げます。また、Arduinoからエンコーダのカウント数を受け取ることでロボットのオドメトリ座標を計算し、/odomを生成しPublishします。
Arduinoとの通信はUDP通信にて実現します。ロボット内のEdgeルータに対して、有線Ethernetケーブルまたは、WiFiに接続することでArduinoと通信することができます。Arduinoドライバで受付IPを指定し、そのIPに対してUDP信号を投げます。デフォルトでは、192.168.11.216に対して投げます。必要に応じて変更してください。

# Requirement
- OS: Ubuntu 20.04.4 LTS
- ROS Distribution: ROS2 Foxy Fitzroy

# Installation
ROS2環境がない場合は[ROS2 Documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)を参照しROS2をインストールしてください。

ROS2のワークスペース内でgit cloneしたのち、colcon buildしてください。
~~~
$ cd ~/your/ros_workspace/ros2_ws/src
$ git clone https://github.com/CuboRex-Development/cugo-ros-controller.git
$ cd ../..
$ colcon build
$ source ~/your/ros_workspace/ros2_ws/install/local_setup.bash
~~~

# Usage

###  (推奨) ros2 launchを用いた起動方法
下記のコマンドで起動します。ros2 launchを用いて起動する際、いくつかのパラメータを指定することができます。詳細は[Parameters](#parameters)の項目を参照してください。
~~~
$ ros2 launch cugo_ros2_control cugo_ros2_control_launch.py
~~~

### ros2 runを用いた起動方法

次のコマンドで起動します。
~~~
$ ros2 run cugo_ros2_control cugo_ros2_control
~~~

# Topics and Parameters
## Published Topics
- /odom ([nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
- /tf ([tf2_msgs/msg/TFMessage](https://docs.ros2.org/latest/api/tf2_msgs/msg/TFMessage.html))

## Subscribed Topics
- /cmd_vel ([geometry_msgs/msg/Twist](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html))

## Parameters
- ~ODOMETRY_DISPLAY (boolean, default: True)
  - オドメトリの表示切替フラグ
- ~PARAMETERS_DISPLAY (boolean, default: False)
  - パラメータの表示切替フラグ
- ~TARGET_RPM_DISPLAY (boolean, default: True)
  - RPMの表示切替フラグ
- ~SENT_PACKET_DISPLAY (boolean, default: False)
  - 送信パケットの表示切替フラグ
- ~RECV_PACKET_DISPLAY (boolean, default: True)
  - 受信パケットの表示切替フラグ
- ~READ_DATA_DISPLAY (boolean, default: True)
  - 受信パケットから抽出したデータの表示切替フラグ
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
- ~source_port (int, default: 8888)
  - ROSノードの通信受付ポート番号
- ~timeout (float, default: 0.05)
  - 通信タイムアウトまでの時間[sec]
- ~tread (float, default: 0.380)
  - トレッド幅[m]
- ~wheel_radius_l (float, default: 0.03858)
  - 左タイヤ半径[m]
- ~wheel_radius_r (float, default: 0.03858)
  - 右タイヤ半径[m]

上記のパラメータはlaunchファイルで設定されています。

# UDP Protocol
CuGo-ROS-ArduinoDriverと、ヘッダ8バイト・ボディ64バイトの合計72バイトから構成されるデータを通信しています。
ボディデータに格納されるデータの一覧は以下の通りになります。
なお、扱うデータは今後拡張する予定です。

### Arduinoドライバへの送信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|--------------------
TARGET_RPM_L   | float      | `4`             | 0                           | RPM指令値(左モータ)
TARGET_RPM_R   | float      | `4`             | 4                           | RPM指令値(右モータ)


### Arduinoドライバからの受信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|-----------------
RECV_ENCODER_L | float      | `4`             | 0                           | 左エンコーダのカウント数
RECV_ENCODER_R | float      | `4`             | 4                           | 右エンコーダのカウント数


# Note

クローラ走行の振動が非常に大きいので、RJ45端子のEthernetケーブルでの通信 / WiFi接続による通信をお勧めします。　　
シリアル通信ものちに対応予定です。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
