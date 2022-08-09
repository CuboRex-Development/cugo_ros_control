# cugo-ros-controller

CuGoをROSで制御する際、ROS開発キットに付属するArduinoに対して制御指令を送り、エンコーダの読み取り結果を受け取るノードです。ROSTopicの/cmd_velをSubscribeし、/odomをPublishします。セットでArduinoドライバと同時に使用します。  
Arduinoドライバのリポジトリはこちら：  
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
 


設定できる項目は以下の通りです。
~~~
// シリアル通信での情報の表示有無
bool UDP_CONNECTION_DISPLAY = false;
bool ENCODER_DISPLAY = true;
bool PID_CONTROLL_DISPLAY = false;
bool FAIL_SAFE_DISPLAY = true;

// Ethernet Shield に印刷されている6桁の番号を入れてください。なお、ロボット内ローカル環境動作なので、そのままでもOK。
byte mac[] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00};  // お持ちのArduinoShield相当の端末のアドレスを記入

// ROSアプリケーションと同じ値にしてください。
IPAddress ip(192, 168, 8, 216);     // Arduinoのアドレス。LAN内でかぶらない値にすること。
unsigned int localPort = 8888;      // 8888番ポートを聞いて待つ

// PID ゲイン調整
// L側
//const float L_KP = 1.5;  // CuGoV3
//const float L_KI = 0.02; // CuGoV3
//const float L_KD = 0.1;  // CuGoV3
const float L_KP = 1.0;    // MEGA
const float L_KI = 0.06;   // MEGA
const float L_KD = 0.1;    // MEGA

// R側
const float R_KP = 1.0;    // MEGA
const float R_KI = 0.06;   // MEGA
const float R_KD = 0.1;    // MEGA
//const float R_KP = 1.5;  // CuGoV3
//const float R_KI = 0.02; // CuGoV3
//const float R_KD = 0.1;  // CuGoV3

// ローパスフィルタ
const float L_LPF = 0.95;
const float R_LPF = 0.95;

// 回転方向ソフトウェア切り替え
const bool L_reverse = true;
const bool R_reverse = false;
~~~
 
# Note
 
クローラ走行の振動が非常に大きいので、RJ45端子のEthernetケーブルでの通信 / WiFi接続による通信をお勧めします。　　
シリアル通信ものちに対応予定です。
 
 
# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
