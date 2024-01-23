# cugo_ros_control
![image](https://github.com/CuboRex-Development/cugo_ros_control/assets/97714660/96b0c630-a114-4e7f-a091-72909d151495)


ROS開発キット/クローラロボット開発プラットフォームに付属するマイコンに対して制御指令を送り、エンコーダの読み取り結果を受け取るノードです。ROS topicの`/cmd_vel`をSubscribeし、`/odom`をPublishします。セットで[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_arduinodriver.git)使用します。


ROS 2 Humbleでディレクトリ構成が変わりました。Foxyをお使いの方は[foxy_devel](https://github.com/CuboRex-Development/cugo_ros_control/tree/foxy-devel) branchをご参照ください。


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
Subscribeした`/cmd_vel`の速度ベクトルになるような仮想車輪L/Rの回転数を計算します。計算した回転数をマイコンに送信します。また、[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)が書き込まれたマイコンからエンコーダのカウント数を受け取ります。カウント数からロボットのオドメトリを計算し、`/odom`を生成しPublishします。

<img width="1142" alt="cugo_ros_control_archi" src="https://github.com/CuboRex-Development/cugo_ros_control/assets/22425319/21e0d954-87ec-436d-9f98-5f8bf35706f9">


#### 対応製品
CuboRex製品では、
* ROS開発キット CuGo V3
* クローラロボット開発プラットフォーム CuGo V4
* クローラロボット開発プラットフォーム CuGo V3i

でお使いいただけます。それぞれ使用するコードが異なることがありますので、下記表からご参照ください。

ここでは、“クローラロボット開発プラットフォーム CuGo V4”と“クローラロボット開発プラットフォーム CuGo V3i”は“クローラロボット開発プラットフォーム”と総称します。


製品名|ROSパッケージ|マイコンスケッチ
-----------|-----------------|-----------------------------
ROS開発キット|このページ|[ArduinoUNO用](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp)                            
クローラロボット開発プラットフォーム|このページ|[RaspberryPiPico用](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)  

#### ROS開発キットの場合
ご購入時点でセットアップ済みですので、そのままROSパッケージの実行をしてください。

このパッケージは付属のArduinoUNOとUDP通信します。ロボット内のEdgeルータに対して、有線Ethernetケーブルまたは、WiFiに接続することでArduinoUNOと通信することができます。[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp)でマイコン側のIPアドレスを指定し、そのIPアドレスに対してUDP信号を送信します。マイコン側のスケッチでは、192.168.11.216をデフォルトにしています。必要に応じてlaunchファイルを変更してください。

付属のArduinoUNOを書き換えてしまった場合は[こちらのスケッチ](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/uno-udp)を書き込んでください。

スケッチの書き換えはROS PCである必要性はありません。

#### クローラロボット開発プラットフォームの場合
付属のRaspberryPiPicoと通信します。
付属のRaspberryPiPicoに[こちらのスケッチ](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)を書き込み、ROS PCとRaspberryPiPicoをUSBケーブルで接続してください。
その後ROSパッケージを実行してください。自動で通信開始します。


スケッチの書き換えはROS PCである必要性はありません。

                 
# Requirement
- OS: Ubuntu 22.04.4 LTS
- ROS Distribution: ROS 2 Humble Hawksbill

# Installation
ROS 2環境がない場合は[ROS 2 Documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)を参照しROS 2をインストールしてください。

IMUのデータ取得に使用している[witmotion_IMU_ros](https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2)のbuildにはver3.19以上のCMakeが必要です。バージョンが低い場合は、[Installing CMake](https://cmake.org/install/)を参考にCMakeをインストールしてください。

ROS 2のワークスペース内でgit cloneしたのち、colcon buildしてください。
~~~
$ cd ~/your/ros_workspace/ros2_ws/src
$ git clone https://github.com/CuboRex-Development/cugo_ros_control.git
$ cd ../..
$ colcon build --symlink-install
$ source ~/your/ros_workspace/ros2_ws/install/local_setup.bash
~~~

# Usage

下記のコマンドでcugo_ros_controlノードが起動します。ros2 launchを用いて起動する際、いくつかのパラメータを指定することができます。詳細は[Parameters](#parameters)の項目を参照してください。

#### ROS開発キット CuGo V3の方
~~~
$ ros2 launch cugo_ros2_control cugov3_ros2_control_launch.py
~~~

#### クローラロボット開発プラットフォーム CuGo V4の方
付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。

~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 755 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control cugov4_ros2_control_launch.py
~~~

#### クローラロボット開発プラットフォーム CuGo V3iの方

付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。
~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 755 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control cugov3i_ros2_control_launch.py
~~~



# Topics and Parameters
## Published Topics
- `/odom` ([nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
- `/tf` ([tf2_msgs/msg/TFMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html))

## Subscribed Topics
- `/cmd_vel` ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))

## Parameters
- `ODOMETRY_DISPLAY (boolean, default: true)`
  - オドメトリ表示の有無（trueで表示）
- `PARAMETERS_DISPLAY (boolean, default: false)`
  - 設定したパラメータを起動時に表示（trueで表示）
- `TARGET_RPM_DISPLAY (boolean, default: true)`
  - 目標RPM表示の有無（trueで表示）
- `SENT_PACKET_DISPLAY (boolean, default: false)`
  - 送信パケットのデバッグ表示の有無（trueで表示)
- `RECV_PACKET_DISPLAY (boolean, default: true)`
  - 受信パケットのデバッグ表示の有無（trueで表示）
- `READ_DATA_DISPLAY (boolean, default: true)`
  - 受信パケットからデコードした数値の表示の有無（trueで表示）
- `arduino_addr (string, default: 192.168.11.216)`
  - Arduinoドライバの通信受付IPアドレス
- `arduino_port (int, default: 8888)`
  - Arduinoドライバの通信受付ポート番号
- `encoder_max (int, default: 2147483647)`
  - エンコーダ最大カウント
  - マイコン側のカウンタがオーバーフローする値を設定
- `encoder_resolution (int, default: 2048)`
  - エンコーダ分解能
- `odom_child_frame_id (string, default: base_link)`
  - オドメトリ子フレームID
- `odom_frame_id (string, default: odom)`
  - オドメトリフレームID
- `reduction_ratio (float, default: 1.0)`
  - モータ軸端からの減速比
- `source_port (int, default: 8888)`
  - ROSノードの通信受付ポート番号
- `timeout (float, default: 0.05)`
  - 通信タイムアウトまでの時間[sec]
- `tread (float, default: 0.380)`
  - トレッド幅[m]
- `wheel_radius_l (float, default: 0.03858)`
  - 左クローラの仮想タイヤ半径[m]
- `wheel_radius_r (float, default: 0.03858)`
  - 右クローラの仮想タイヤ半径[m]
- `comm_type (string, default: UDP)`
  - 通信をUDPでするかUSBでするか
- `serial_port (string, default: /dev/ttyACM0)`
  - USB-Serialで通信するポートを指定
- `serial_baudrate (int, default: 115200)`
  - USB-Serial通信のボーレートを設定

上記のパラメータはlaunchファイルで設定されています。

# Protocol
[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)と、ヘッダ8バイト・ボディ64バイトの合計72バイトから構成されるデータを通信しています。
ボディデータに格納されるデータの一覧は以下の通りになります。
なお、扱うデータは今後拡張する予定です。

### Arduinoドライバへの送信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|--------------------
TARGET_RPM_L   | float      | 4             | 0                           | RPM指令値(左モータ)
TARGET_RPM_R   | float      | 4             | 4                           | RPM指令値(右モータ)


### Arduinoドライバからの受信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|-----------------
RECV_ENCODER_L | float      | 4             | 0                           | 左エンコーダのカウント数
RECV_ENCODER_R | float      | 4             | 4                           | 右エンコーダのカウント数


# Note

ご不明点がございましたら、[issues](https://github.com/CuboRex-Development/cugo_ros_control/issues)にてお問い合わせください。回答いたします。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
