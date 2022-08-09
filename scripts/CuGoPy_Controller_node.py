#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
#import serial
import time
import socket
from math import sin, cos, pi

#from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Vector3
from nav_msgs.msg import Odometry


class CuGo_Controler():
    def __init__(self):
        # Subscriberの作成
        self.sub = rospy.Subscriber('/cmd_vel', Twist, self.callback)
        # Publisherの作成
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        # tfの作成
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        # ノード共通値の初期化
        self.vector_v       = 0.0
        self.vector_omega   = 0.0
        self.target_rpm_l          = 0.0
        self.target_rpm_r          = 0.0
        self.recv_encoder_l = 0
        self.recv_encoder_r = 0
        self.last_recv_encoder_l = 0    # signed int
        self.last_recv_encoder_r = 0    # signed int
        self.odom_x         = 0.0
        self.odom_y         = 0.0
        self.odom_z         = 0.0
        self.odom_roll      = 0.0
        self.odom_pitch     = 0.0
        self.odom_yaw       = 0.0
        self.odom_twist_x   = 0.0
        self.odom_twist_y   = 0.0   # メカナムホイール等直接真横に移動できる場合
        self.odom_twist_yaw = 0.0
        self.Vx_dt          = 0.0
        self.Vy_dt          = 0.0   # メカナムホイール等直接真横に移動できる場合
        self.Theta_dt       = 0.0
        self.start_serial_comm = False
        # TODO: ROSLaunchで外部設定ができるように
        self.device_name    = '/dev/Arduino'
        self.baudrate       = 115200
        self.timeout        = 0.05  # 10hzで回す場合
        self.wheel_radius_l = 0.03858   #初期値はCuGO V3の値
        self.wheel_radius_r = 0.03858   #初期値はCuGO V3の値
#        self.tread          = 0.380     #初期値はCuGO V3の値
        self.reduction_ratio = 1.0      #初期値はCuGo V3の値
#        self.wheel_radius_l  = 0.030375  #CuGO MEGAの値
#        self.wheel_radius_r  = 0.030375  #CuGO MEGAの値
        self.tread           = 0.635     #CuGO MEGAの値
        #self.reduction_ratio = 0.3333    #CuGO MEGAの値

        self.encoder_resolution = 2048
        self.encoder_max    = 2147483647 # -2147483648 ~ 2147483647(Arduinoのlong intは32bit)
        
        self.not_recv_cnt   = 0
        self.stop_motor_time = 500 #NavigationやコントローラからSubscriberできなかったときにモータを止めるまでの時間(ms)
        #self.arduino_addr = ('192.168.1.177', 8888)
        self.arduino_addr = ('192.168.8.216', 8888) 
        self.UDP_BUFF = 256
        self.send_str = ''
        self.recv_str = ''


    def callback(self, msg):
        # callback関数の処理をかく
        self.vector_v = msg.linear.x
        self.vector_omega = msg.angular.z
        # TODO: 通信ロストを検知。異常状態の対応を決める
        # ex)ゼロ埋め、異常フラグ管理
        self.subscribe_time = rospy.get_time()


    def publish(self):
        odom_quat = tf.transformations.quaternion_from_euler(0,0,self.odom_yaw)
        # TODO roslaunch
        self.odom_broadcaster.sendTransform( \
            (self.odom_x, self.odom_y, 0),  \
            odom_quat,  \
            self.recv_time, \
            "base_link",    \
            "odom"  \
        )

        odom = Odometry()
        odom.header.stamp = self.recv_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = Pose(Point(self.odom_x, self.odom_y, 0.0), Quaternion(*odom_quat))
        odom.twist.twist = \
            Twist(Vector3(self.odom_twist_x , 0, 0), \
                  Vector3(0, 0, self.odom_twist_yaw))

        self.view_odom()
        self.pub.publish(odom)


    def view_odom(self):
        print('in_twist: ',self.vector_v, ', ', self.vector_omega)
        print('out_twist: ',self.odom_twist_x, ', ', self.odom_twist_yaw)
        print('odom: ', self.odom_x,', ',self.odom_y,', ',self.odom_yaw)


    def view_init(self):
        print('view_init')


    # 工事中。22年末までに追加予定
    def init_serial(self):
        # TODO: PermissionDeniedの場合にやさしく教えてあげる
        # TODO: シリアル通信の値が読めない現象について
        #前回セッションがメモリに残っている場合、シリアルバッファを読み続けてしまう。最後に格納された値をずっと読み続ける。1秒ほどでシリアルデバイスをクローズ→オープンにしてもGUIツールでは決まって10秒ほどシリアルバッファを読み続けて（しかも処理できる速度の限り）、プロセスを占有する。おそらくこれが原因でsshの通信もブロッキングされ操作不能になる。
        self.ser = serial.Serial(self.device_name, self.baudrate, timeout=self.timeout)
        self.ser.reset_input_buffer()
        # シリアル受信時間を初期化

        #self.ser.setDTR(False)
        print('serial init')
        time.sleep(2) # プログラム再起動時に間髪入れずに通信を始めるとうまく行かない

        # TODO: 初回起動で、デバイスに権限を与えた直後に落ちることがある。
        # recv_str = recv_str.decode()
        # UnicodeDecodeError: 'utf-8' codec can't decode byte 0xfb in position 0: invalid start byte


    def init_time(self):
        # 送受信時間確認　これはUDPのときに作り直す
        #self.recv_time      = rospy.Time.now()
        #self.last_recv_time = rospy.Time.now()
        self.recv_time      = rospy.Time.now()
        self.last_recv_time = rospy.Time.now()
        self.subscribe_time = rospy.get_time()
       

    def init_UDP(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    def close_UDP(self):
        print('UDP port close...')
        self.sock.close()


    def serial_send_cmd(self):
        send_str = str(self.target_rpm_l) + ',' + str(self.target_rpm_r) + '\n'
        #print(send_str)
       # self.ser.write(str.encode(send_str))
        print('send rpm:',self.target_rpm_l,', ',self.target_rpm_r)
        
        #time.sleep(0.01)
        #self.ser.flush()

    def UDP_send_string_cmd(self):
        print('UDP_send_cmd')
        print('send cmd:', self.send_str)
        self.UDP_send_time = rospy.get_time() #UDPの一往復の時間を図る
        send_len = self.sock.sendto(self.send_str.encode('utf-8'), self.arduino_addr)
        

    def serial_reciev_state(self):#UDPのいまは使わない
        # TODO: 本当に受信したときだけodomに反映できているか再確認するべし
        # 無駄にodomに積算されないか確認
        recv_str = self.ser.readline()
        #print(recv_str_len.size())
        # TEST: UTF-8でデコードできない場合は文字列を出し、スキップする

        #try:
        #    recv_str = recv_str.decode()
        #except UnicodeDecodeError:
        #    recv_str = '0.0, 0.0'
        #    print('不正な値を受信.走行停止.')

        print('str:',recv_str)
        recv_str = recv_str.decode()
        #recv_str.strip() #改行文字のOSの違いをここで緩衝
        recv_str_len = len(recv_str)
        print('str:',recv_str)
        print('str len:',recv_str_len)

        if recv_str_len > 1 :
            str_arr = recv_str.split(',')
            if len(str_arr) >= 4 :
                # TODO: intにキャストできないときに落ちる(Arduinoのイニシャル通信)
                self.recv_encoder_l = int(str_arr[0])
                self.recv_encoder_r = int(str_arr[1])

                # 開発用
                print('reciev_rpm: ', str_arr[2], ', ',str_arr[3])

                self.last_recv_time = self.recv_time
                self.recv_time = rospy.Time.now()

                if self.start_serial_comm == False :
                    print('通信を開始できたヨ')
                    self.start_serial_comm = True
        
        else :
            print('受信できてないヨ！')
            self.start_serial_comm = False

        self.ser.flushInput() # serial read バッファの残留データで走行しないことがわかったため。
        #print(recv_str)
        # TODO: マイコン側の返答がないとき何が問題なのかわからない
        # 1.length=0　データが来ていないままタイムアウトになったとき
        # 2.length=1　改行文字のみが来ているとき
        # 3.length=-1 むしろシリアルの機能内で弾かれて終了するべき
        #テストケース
        #・マイコンのリセットを押し続けて値が来ない状態を検知できる
        #・usbを抜けたことを検知して、代替値を入れる
        #・再び刺されたことを検知して、自動的に接続し値を入れる


    def check_overflow(self, diff_, max_):
        # マイコン側のintが16bitなので補正する →32bitに変更済み
        # 上限/下限で16bit分一周回るので改めて足したり引いたりしてあげる
        # 上限
        if diff_ > max_ * 0.9:
            diff_ = diff_ - max_ * 2
            print('Overflow: ',diff_)
            print('Overflow max!')
            return diff_
        # 下限
        if diff_ < -max_ * 0.9:
            diff_ = diff_ + max_ * 2
            print('Overflow: ',diff_)
            print('Overflow min!')
            return diff_

        return diff_
    

    def count2twist(self):
        print('\ncount2twist')
        diff_time = self.recv_time.to_sec() - self.last_recv_time.to_sec() 
        print("diff_time:" + str(diff_time))
        if diff_time != 0.0: # 0割防止 0割だと何もしないまま値が受け取れてない扱いダメ！
            # UDP通信が完了してからやる。テストを別関数にしたのでそれで検証。
            print("calc twist")
            self.last_recv_time = self.recv_time
            count_diff_l = self.recv_encoder_l - self.last_recv_encoder_l
            count_diff_r = self.recv_encoder_r - self.last_recv_encoder_r
            
            # 3カウント以下はノイズと判定しこのループは無視する
            # 片方だけ無視とかは不具合の温床なので、優先度低めで対応
            # この対応をすることで積算誤差が貯まる時間を延命できる
            #if (count_diff_l < 3) or (-3 < count_diff_l): return
            #if (count_diff_r < 3) or (-3 < count_diff_l): return
                
            self.last_recv_encoder_l = self.recv_encoder_l
            self.last_recv_encoder_r = self.recv_encoder_r

            #print('before check:',count_diff_l)
            # マイコン側のオーバーフロー対策
            count_diff_l = self.check_overflow(count_diff_l, self.encoder_max)
            count_diff_r = self.check_overflow(count_diff_r, self.encoder_max)
            
            #print('after check:',count_diff_l)

            vl = count_diff_l / (self.encoder_resolution * self.reduction_ratio) * \
                 2 * self.wheel_radius_l * math.pi
            vr = count_diff_r / (self.encoder_resolution * self.reduction_ratio) * \
                 2 * self.wheel_radius_r * math.pi
            #print('rpm: ', (count_diff_l / diff_time) / self.encoder_resolution * 60)
            #print('encoder_max:', self.encoder_max)

            # 微小時間で送られたカウンタ差分なので、速度も角度も微小時間
            #print('count_l:', count_diff_l,', vl:', vl)
            self.Vx_dt = (vl + vr) / 2
            self.Theta_dt = (vr - vl) / self.tread # 半時計周りが正

            self.odom_twist_x   = self.Vx_dt / diff_time
            self.odom_twist_yaw = self.Theta_dt / diff_time

        else :
            # TODO:値が受け取れていないフラグ処理
            #print('値が受け取れてないよヨ')

            # 22/7/27テスト用
            print('22/7/27 odom calc test')
            self.calc_count_to_vec() # これは使わない


    def calc_odom(self):
        self.odom_x += self.Vx_dt * cos(self.odom_yaw) - self.Vy_dt * sin(self.odom_yaw)
        self.odom_y += self.Vx_dt * sin(self.odom_yaw) - self.Vy_dt * cos(self.odom_yaw)
        self.odom_yaw += self.Theta_dt


        # Serialのみ
    def calc_count_to_vec(self):
        #通常は受信時間だが、テストとして10Hzで正確に回ったケースとして使用
        #diff_time = self.recv_time.to_sec() - self.last_recv_time.to_sec() 
        diff_time = 0.1
        self.last_recv_time = self.recv_time
        print('diff_time', diff_time)
        
        #7/27テスト用数値
        self.recv_encoder_l += (self.target_rpm_l / 60) * self.encoder_resolution
        self.recv_encoder_r += (self.target_rpm_r / 60) * self.encoder_resolution
        print('encoder L/R:',self.recv_encoder_l, self.recv_encoder_r)

        count_diff_l = self.recv_encoder_l - self.last_recv_encoder_l
        count_diff_r = self.recv_encoder_r - self.last_recv_encoder_r
        print('count diff L/R:',count_diff_l,count_diff_r)

        # 3カウント以下はノイズと判定しこのループは無視する
        # 片方だけ無視とかは不具合の温床なので、優先度低めで対応
        # この対応をすることで積算誤差が貯まる時間を延命できる
        #if (count_diff_l < 3) or (-3 < count_diff_l): return
        #if (count_diff_r < 3) or (-3 < count_diff_l): return

        self.last_recv_encoder_l = self.recv_encoder_l
        self.last_recv_encoder_r = self.recv_encoder_r

        #print('before check:',count_diff_l)
        # マイコン側のオーバーフロー対策
        count_diff_l = self.check_overflow(count_diff_l, self.encoder_max)
        count_diff_r = self.check_overflow(count_diff_r, self.encoder_max)

        #print('after check:',count_diff_l)

        vl = count_diff_l / self.encoder_resolution * \
             2 * self.wheel_radius_l * math.pi
        vr = count_diff_r / self.encoder_resolution * \
             2 * self.wheel_radius_r * math.pi
        print('rpm: ', (count_diff_l / diff_time) / self.encoder_resolution * 60)
        print('encoder_max:', self.encoder_max)

        # 微小時間で送られたカウンタ差分なので、速度も角度も微小時間
        print('count_l:', count_diff_l,', vl:', vl)
        self.Vx_dt = (vl + vr) / 2
        self.Theta_dt = (vr - vl) / self.tread # 半時計周りが正

        self.odom_twist_x   = self.Vx_dt / diff_time
        self.odom_twist_yaw = self.Theta_dt / diff_time


    def twist2rpm(self):
        omega_l = self.vector_v / self.wheel_radius_l \
                  - self.tread * self.vector_omega / (2 * self.wheel_radius_l)
        omega_r = self.vector_v / self.wheel_radius_r \
                  + self.tread * self.vector_omega / (2 * self.wheel_radius_r)

        self.target_rpm_l = omega_l * 60 / (2 * math.pi)
        self.target_rpm_r = omega_r * 60 / (2 * math.pi)

        print('%s, %s'%(self.target_rpm_l,self.target_rpm_r))


    def check_stop_cmdvel(self):
        #print('なにも受け取らないまま500msたったよ')
        print('\ncheck_stop_cmdvel')
        #print(rospy.Time.now().to_sec - self.subscribe_time.to_sec)
        subscribe_duration = rospy.get_time() - self.subscribe_time
        if(subscribe_duration > (self.stop_motor_time / 1000)):
            print('/cmd_vel disconnect...\nset target rpm 0.0')
            self.vector_v = 0.0
            self.vector_omega = 0.0

        #print(subscribe_duration)
        #rospy.loginfo(from_subscribe_time.nsec)

    def send_rpm_MCU(self):
        #print('ここでMCUに指示rpmをUDPで送るよ')
        print('\nsend_rpm_MCU')
        #self.serial_send_string_cmd()
        self.send_str = str(self.target_rpm_l) + ',' + str(self.target_rpm_r) + '\n'
        print(self.send_str)

        self.UDP_send_string_cmd()

        


    def recv_count_MCU(self):
        #print('ここでMCUからエンコーダのカウント数をもらうよ')
        print('\nrecv_count_MCU')
        self.sock.settimeout(0.05)

        try:
            recv_str_bin, self.arduino_addr = self.sock.recvfrom(self.UDP_BUFF)
        except socket.timeout:
            print('socket timeout!')
            self.sock.settimeout(None)

        else:
            self.UDP_recv_time = rospy.get_time() #UDPの一往復の時間を図る
            print('UDP time:', (self.UDP_recv_time - self.UDP_send_time)) #UDPの一往復の時間を図る

            # ベクトル計算用の時間を計測
            self.recv_time = rospy.Time.now()
            self.recv_str = recv_str_bin.decode(encoding='utf-8')
            
            # 送られてきた文字列が正しいか確認
            print('recv_cmd:',self.recv_str)
            str_arr = self.recv_str.split(',')
            if len(str_arr) >= 2 :
                # TODO: intにキャストできないときに落ちる(Arduinoのイニシャル通信)
                self.recv_encoder_l = int(str_arr[0])
                self.recv_encoder_r = int(str_arr[1])

                # 開発用
                print('reciev_count:', str_arr[0], ',',str_arr[1])

            # タイムアウト設定時間を戻す
            self.sock.settimeout(None)
           

        # テスト用に/cmd_velの回転がそのままカウンタになったとする
        

    def odom_publish(self):
        #print('ここで他のROSノードに/odomを送るよ')
        print('\nodom_publish')
        self.calc_odom()
        self.publish()
        print('\n\n')

        
    def node_shutdown(self):
        node.close_UDP()
        print('node_shutdown')

        

if __name__ == '__main__':

    try:
        rospy.init_node('CuGo_Controler')
        node = CuGo_Controler()
        rate = rospy.Rate(10)

        # マイコンとのシリアル通信開始
        #node.init_serial()
        node.init_time()
        node.init_UDP()

        while not rospy.is_shutdown():

            node.check_stop_cmdvel()
            node.twist2rpm()
            node.send_rpm_MCU()
            node.recv_count_MCU()
            node.count2twist()
            node.odom_publish()

            rate.sleep()

        print('\n')
        node.node_shutdown()

    except rospy.ROSInterruptException: 
        node.node_shutdown()
        # TODO: USBを引っこ抜いたとき、例外に入る処理をしていないためクローズしない
        pass
