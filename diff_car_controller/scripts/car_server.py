#!/usr/bin/env python
# -*-coding:UTF-8 -*-
"""
car server文件，是car类的client类，执行下列功能：
    - 打开 bus
    - 将 bus 传递给 car 类
    - 在 car 的run_mode模式下
        * 监听 cmd_vel消息, 并执行
        * 跟新 car 的 odom 信息 ： bus 获取轮子信息   car 根据轮子信息，跟新车辆 odom
    - 在 car 的config_mode模式下
        * 等待状态，不执行命令

    轮子的控制使用的是cmd_vel，不使用action_server；因此使用简单的topic和service机制
"""

from car_control import car
from ZL_motor_control import ZL_motor_control
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import tf
from diff_car_controller.srv import *

import threading
import time
try:
    import _thread
except:
    import thread as _thread
    
mutex = threading.Lock()

def odom_puber(odom_info,puber):
    msg = Odometry()
    msg.header.frame_id = 'odom_link'
    msg.child_frame_id = 'base_link'
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg.header.seq = msg.header.seq + 1
        msg.header.stamp = rospy.Time.now()
        msg.pose.pose.position.x = odom_info['x']
        msg.pose.pose.position.y = odom_info['y']
        msg.pose.pose.position.z = 0
        odom_qua = tf.transformations.quaternion_from_euler(0, 0, odom_info['theta'])
        msg.pose.pose.orientation.x = odom_qua[0]
        msg.pose.pose.orientation.y = odom_qua[1]
        msg.pose.pose.orientation.z = odom_qua[2]
        msg.pose.pose.orientation.w = odom_qua[3]
        msg.twist.twist.linear.x = odom_info['v']
        msg.twist.twist.angular.z = odom_info['w']
        puber.publish(msg)
        br.sendTransform((odom_info['x'],odom_info['y'],0),odom_qua,rospy.Time.now(),"base_link","odom_link")
        rate.sleep()

def vel_callback(msg,arg):
    start = time.time()
    diff_car = arg[0]
    #print(msg)
    # 处理速度，取决于/cmd_vel的发布速度。
    if diff_car.isRunMode:
        v = msg.linear.x
        #v = 5  #rad/s
        w = msg.angular.z
        #w = 5
        #print("v",v,"w",w)
        # 使用isSending避免主线程在本线程发送数据的时候，调用update函数，破坏本线程调用update函数。
        # 由于不关心，发送是否成功，因此，在此决定
        #diff_car.isSending = True
        #if  v !=0 or w !=0:
            #print("let's move the car")
            #mutex.acquire()
        diff_car.set_car_vel(v,w)
            #mutex.release()
        diff_car.isSending = False
    #timepass = start - time.time()
    #print("time pass is:",timepass)
        
 

    # SetMode  未定义srv类型：其内容包括：
    # ---req    int request : run_mode:1   config_mode:0
    # ---Res    Bool   answer :  True       Fasle
def set_mode_callback(req,diffcar):
    if req.request == 'run_mode':
        diffcar.run_mode()
        rospy.loginfo("进入运行模式！")
        return SetModeResponse(True)
    else:
        diffcar.config_mode()
        rospy.loginfo("进入配置模式！")
        return SetModeResponse(True)

def mode_server(args):
    diffcar = args
    s = rospy.Service("set_mode_server",SetMode,set_mode_callback,(diffcar))
    rospy.loginfo("mode server opened!")
    rospy.spin()

if __name__ == '__main__':

    # 初始化节点
    # 建立 bus 以及 car 的 parameter server 参数
    # 初始化bus
    # 初始化car
    # 建立 odom 发布的 线程
    # 建立 修改 car 模式的 service
        # run_mode service
        # config_mode service
    # 默认进入 run_mode
    # 判断 car 处于 run_mode 还是 config_mode
    # run_ mode
        # 判断 cmd_vel 队列是否有消息到达
            # 有消息，执行 消息回调函数
            # 无消息，执行 read status函数
    # config_mode
        # 不做任何动作

    # 初始化节点
    rospy.init_node("car_server",anonymous=True)

    # 设置parameter server
    # bus params
    if not rospy.has_param("~bus_channel"):
        rospy.set_param("~bus_channel","/dev/wheels_ZL")
    if not rospy.has_param("~bus_baudrate"):
        rospy.set_param("~bus_baudrate",115200)
    if not rospy.has_param("~bus_bitrate"):
        rospy.set_param("~bus_bitrate",115200)
    if not rospy.has_param("~bus_type"):
        rospy.set_param("~bus_type",'serial')
    if not rospy.has_param("~bus_id_list"):
        rospy.set_param("~bus_id_list",[1,2])
    # car params
    if not rospy.has_param("~wheel_diameters"):
        rospy.set_param("~wheel_diameter",0.100)
    if not rospy.has_param("~wheel_distance"):
        rospy.set_param("~wheel_distance",0.100)

    # 获取设置的参数
    bus_channel = rospy.get_param("~bus_channel")
    bus_baudrate = rospy.get_param("~bus_baudrate")
    bus_bitrate = rospy.get_param("~bus_bitrate")
    bus_type = rospy.get_param("~bus_type")
    bus_id_list = rospy.get_param("~bus_id_list")

    wheel_diameter = rospy.get_param("~wheel_diameter")
    wheel_distance = rospy.get_param("~wheel_distance")

    # 初始化 bus  car
    bus = ZL_motor_control(bus_channel,bus_bitrate,bus_baudrate,bus_id_list,bus_type)
    diff_car = car(wheel_diameter,wheel_distance,bus,bus_id_list)
    
    # 默认进入 RunMode
    diff_car.run_mode()

    # 建立odom的发布线程
    odom_publisher = rospy.Publisher('/odom',Odometry,queue_size=10)
    
    # 建立 cmd_vel 的subscriber
    rospy.Subscriber('/cmd_vel',Twist,vel_callback,(diff_car,))
    
    try:
        odom_thread = _thread.start_new(odom_puber, (diff_car.odom, odom_publisher))
    except :
        rospy.loginfo("thread creation failed! program exit!")
        exit(0)
    finally:
        rospy.loginfo("odom thread created！ begin to pub odom info and transform from odom_link to base_link")

    # mode service 线程
    # 建立一个service，用于外部程序控制本程序的runmode 或者 config mode 
    _thread.start_new(mode_server,(diff_car,))

    i = 0
    # 判断 car 处于 run_mode 还是 config_mode
    while not rospy.is_shutdown():
        if diff_car.isRunMode:
            #if i == 0 :
                #rospy.loginfo("car is in run mode!")
            #避免过于频繁的读取（当buffer没有可读取内容时，会出现问题。）
            #rospy.Rate(10)
            # 判断是否有消息需要处理
            #rospy.spinOnce()
            #if not diff_car.isSending:
            #mutex.acquire()
            #i = 1
            diff_car.update_status()  
            #print(diff_car.bus.status)
            #print("update times:",i)
            #i = i + 1
            #mutex.release()
        else:
            # do notiong just update status
            #if i == 1:
                #rospy.loginfo("car is in configure mode!")
            diff_car.update_status()
            #i = 0
