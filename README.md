# cugo-ros-controller

CuGoをROSで制御する際、ROS開発キットに付属するArduinoに対して制御指令を送り、エンコーダの読み取り結果を受け取るノードです。ROSTopicの/cmd_velをSubscribeし、/odomをPublishします。セットでArduinoドライバと同時に使用します。  
Arduinoドライバのリポジトリはこちら： https://github.com/CuboRex-Development/cugo-ros-arduinodriver.git  
English Documents here：   
正式リリースするまでは、beta branchで管理しますので、そちらをご参照ください。
 
# Features
CuGo-ROS-ArduinoDriverに対して、/cmd_velのベクトルを達成するためのロボットのL/Rの回転数を計算し指示を投げます。また、Arduinoからエンコーダのカウント数を受け取ることでロボットのオドメトリ座標を計算し、/odomを生成しPublishします。  
Arduinoとの通信はUDP通信にて実現します。ロボット内のEdgeルータに対して、有線Ethernetケーブルまたは、WiFiに接続することでArduinoと通信することができます。Arduinoドライバで受付IPを指定し、そのIPに対してUDP信号を投げます。デフォルトでは、192.168.8.216に対して投げます。必要に応じて変更してください。

# Requirement
Python3環境を使用します。ライブラリとして以下のものを使用しますが、Python3インストール後、別途ライブラリをインストールする必要はございません。
 
# Installation
ROSのワークスペース内でgit cloneしたのち、catkin_makeしてください。
~~~
$ cd ~/your/ros_workspace/catkin_ws/src
$ git clone ....git
$ cd ../..
$ catkin_make
~~~
 
# Usage
 
実行方法
~~~
$ rosrun CuGoPy_Controller CuGoPy_Controller.py
~~~
→ノード名が旧名なので、"cugo-ros-controller"に後ほど変更します。

Launchで起動すると、joy_nodeやteleop_joy_nodeを同時に起動することができます。
~~~
$ roslaunch CuGoPy_Controller start_ps4_controller.launch
~~~
他に、任意のノードと同時に起動したい場合、こちらのlaunchファイルを編集して使用してください。
~~~
$ roslaunch CuGoPy_Controller start_any_apps.launch
~~~

ROSのインストールなどは後ほど記載します。


画面の表示方法、見られるパラメータに関しては後ほど整理、記載します。（Arduino側と同等に表示する内容を選択できるようにします）

 
# Note
 
クローラ走行の振動が非常に大きいので、RJ45端子のEthernetケーブルでの通信 / WiFi接続による通信をお勧めします。　　
シリアル通信ものちに対応予定です。
 
 
# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
