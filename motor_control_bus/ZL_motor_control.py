#!/usr/bin/env python
#-*_coding:UTF-8-*-
"""
ZL_motor_contorl类是中菱轮毂电机的motor_control_bus的具体实现。
    - bus：使用的是python-can 的serial bus
    - motor_id：包含了该条bus网路中，包含的motor id，即can的从站号
    - frame的收取是python-can的内部行为，不再需要进行判断是否为完整帧。
    - frame的格式使用python-can的Message
"""
# @ step
# 首先是获取python-can bus对象

from __future__ import print_function
import abc
import struct
import time
import can
from can import interfaces
from can import Message
from motor_control_bus import MotorControlBus
import copy
import logging

class ZL_motor_control(MotorControlBus):

    #初始化
    """
        channel：表示串口号
        bitrate:表示波特率或者Can速率
        baudrate:表示波特率
        bustype:表示can bus的类型
    """
    def __init__(self, channel, bitrate, baudrate, id_list, bustype = 'serial',timeout = 0.3):
        self.logger = logging.getLogger("ZL_motor_control_logger")
        self.loghandle = logging.StreamHandler()
        self.logger.addHandler(self.loghandle)
        self.setloggerlevel(logging.DEBUG)
        self.channel = channel
        self.bitrate = bitrate
        self.bustype = bustype
        self.timeout = timeout
        self.id_list = id_list
        #打开bus
        self.open_bus()
        self.status = {}
        self.statuscallback = {}
        #设置轮子状态
        for member in self.id_list:
            self.status[member] = {"Pos":0, "Vel":0, "I":0, "Err":0}
        # 状态反馈报文
        self.statuscallback["Stat_Pos"] =   can.Message(data=[0x00,0xFE,0x00,0x20,0x00,0x00,0x00,0x00])
        self.statuscallback["Stat_Vel_I"] = can.Message(data=[0x00,0xFE,0x00,0x21,0x00,0x00,0x00,0x00])
        # 报警报文
        self.statuscallback["Err_02"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x02, 0x02, 0x02, 0x02])   #过流
        self.statuscallback["Err_04"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x04, 0x04, 0x04, 0x04])   #过压
        self.statuscallback["Err_08"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x08, 0x08, 0x08, 0x08])   #编码器故障
        self.statuscallback["Err_10"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x10, 0x10, 0x10, 0x10])   #过热
        self.statuscallback["Err_20"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x20, 0x20, 0x20, 0x20])   #欠压
        self.statuscallback["Err_40"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x40, 0x40, 0x40, 0x40])   #过载
        self.statuscallback["Err_01"] = can.Message(data=[0x00, 0x0FE, 0x00, 0xEB, 0x01, 0x01, 0x01, 0x01])   #运行状态


    def setloggerlevel(self,level):
        self.logger.setLevel(level)
        self.loghandle.setLevel(level)

    # 打开bus
    def open_bus(self):
        try:
            # 打开方式
            self.logger.debug("self.open_bus()-->打开串口")
            self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel, biterate=self.bitrate)
            self.logger.debug(self.bus)
            #self.bus = can.interfaces.serial.serial_can.SerialBus(channel = self.channel, baudrate = self.baudrate)
        #捕捉异常
        except ValueError as e:
            print(e)
            return False
        finally:
        #没有异常
            return True

    #   关闭bus
    def close_bus(self):
        try:
            #正常关闭，返回True
            self.bus.shutdown()
            return True
        except:
            #关闭异常，返回false
            return False

    """
        id:表示从站id,单个id
        addr:表示寄存器地址，由地址高地位表示addr1,addr2]
        value:表示目标数据，32位数据。
        vale采用大端在前，小段在后表示
        data格式：[0x00 0xfa 0xaddr[0] 0xaddr[1] 0xvalue[0] 0xvalue[1] 0xvalue[2] 0xvalue[3]]
        0x--表示一个字节
        
    """

    # 由于pythoncan的msg与中菱电机的msg不同，因此重写发送和接受msg的函数
    def send(self, msg, timeout=0.3):
        self.logger.debug("self.send()")
        try:
            a_id = struct.pack('>I', msg.arbitration_id)
        except Exception:
            raise ValueError('Arbitration Id is out of range')
        byte_msg = bytearray()
        byte_msg.append(0xAA)
        byte_msg.append(msg.is_extended_id)
        byte_msg.append(msg.is_remote_frame)
        byte_msg.append(msg.dlc)
        for i in range(0, 4):
            byte_msg.append(a_id[i])
        for i in range(0, msg.dlc):
            byte_msg.append(msg.data[i])
        self.bus.ser.write(byte_msg)

    def recv(self, timeout=0.3):
        timepass = 0
        timestamp = time.time()
        while(timepass < timeout):
            try:
                rx_byte = self.bus.ser.read()
                # print('rx_byte is :',ord(rx_byte))
            except:
                # print("rx_byte is None")
                return None
            if len(rx_byte) and ord(rx_byte) == 0xAA:
                # print("begin to get details")
                s = bytearray(self.bus.ser.read())
                # print("extended_id",s)
                extended_id = struct.unpack('?', s)
                s = bytearray(self.bus.ser.read())
                remote_frame = struct.unpack('?', s)
                dlc = ord(self.bus.ser.read())
                s = bytearray(self.bus.ser.read(4))
                arb_id = (struct.unpack('>I', s))[0]
                data = self.bus.ser.read(dlc)
                return Message(timestamp=timestamp / 1000,
                               arbitration_id=arb_id, dlc=dlc, data=data,
                               extended_id=extended_id)
                timepass = time.time()-timestamp
        print("        read failed!")
        return False

    # 装填帧
    # 输入数据帧，返回Message数据
    def ship_frame(self, id, addr, value):
        valuebytes = struct.pack(">i", value)
        data = bytearray([0x00, 0xFA, addr[0], addr[1]])
        data = data + valuebytes
        frame = Message(extended_id=False,data=data)
        frame.arbitration_id = id
        frame.timestamp = time.time()
        frame.dlc = 8
        return frame

    # 发送帧
    # 发送Can.Message类型的数据
    def send_frame(self,frame):
        try:
            self.send(frame,0.2)
            return True
        except ValueError as e:
            print(e)
            return False
        except:
            print("        send failed!")
            return False

    # 同一的发送以及确认过程
    # id_list是从站号   addr:寄存器地址  data_dict:数据字典[从站号：数据] possible_callback:可能的反馈报文  callback_resualt:返回值
    def send_and_confirm(self,id_list,addr,data_dict):
        self.logger.debug("get in self.send_and_confrim ")
        possible_callback={}
        callback_result={}
        for member in id_list :
            data = data_dict[member]
            msg = self.ship_frame(member,addr,data)
            try:
                self.send(msg,timeout=self.timeout)
                self.logger.debug("        send a message in self.send_and_confirm()")
            except:
                print("send failed!")
                return False
            finally:
                # 计算可能的返回报文
                self.logger.debug("        begin to check possible callback in self.send_and_confirm!")
                possible_callback[member] = self.cal_callbackframe(msg)
                # 保存报文信息的列表
                callback_result[member] = -1
                self.check_callback(possible_callback,callback_result)
        for member in id_list:
            if callback_result[member] == False or callback_result[member] == -1:
                return [False, callback_result]
        return True

    # 检查反馈报文
    # 设置检测时间变量
    # 将报文反馈结果存储
    def check_callback(self,possible_callback,callback_result):
        self.logger.debug("in the self.check_callback!")
        starttime = time.time()
        while(True):
            #计算时间长度
            timepass = time.time() - starttime
            #时间长度大于设置时间，判定为失败
            self.logger.debug("        time pass")
            self.logger.debug(timepass)
            if(timepass >= self.timeout):
                return False
            # 在timeout时间内检测到反馈报文
            # 接受报文
            try:
                self.logger.debug("        begin to read a message!")
                recv_msg = self.recv(timeout = self.timeout)
                self.logger.debug("        read a msg in self.check_callback!")
                self.logger.debug(recv_msg)
            except can.CanError as e:
                print(e)
                return False
            # 报文ID
            self.logger.debug("        copy id_list")
            callback_id = copy.deepcopy(self.id_list)
            self.logger.debug(callback_id)
            self.logger.debug("         recv_msg.arbitration_id")
            self.logger.debug(recv_msg.arbitration_id)
            if recv_msg.arbitration_id in callback_id:
                self.logger.debug("        begin to check!")
                member = recv_msg.arbitration_id
                result = self.confirm_callback(possible_callback[member],recv_msg)
                self.logger.debug(result)
                if(result == True):
                    callback_id.remove(member)
                    callback_result[member] = True
                elif (result == False):
                    callback_id.remove(member)
                    callback_result[member] = False
                elif (result == -1):
                    self.read_status(recv_msg)
            if len(callback_id) == 0:
                return callback_result

    # 计算可能的反馈报文，并装到一个字典中
    # ["True","False"]
    def cal_callbackframe(self,frame_sended):
        possiblecallback = {}
        if(frame_sended.data[1]==0xFA):
            frame_sended.data[1]=0xFB
            possiblecallback["True"]=copy.deepcopy(frame_sended)  #正确反馈
            frame_sended.data[1]=0xFD
            possiblecallback["False"]=copy.deepcopy(frame_sended)  #错误反馈
        elif (frame_sended.data[1] == 0xDA):
            frame_sended.data[1]=0xDB
            possiblecallback["True"]=copy.deepcopy(frame_sended)
            frame_sended.data[1]=0xDD
            possiblecallback["False"]=copy.deepcopy(frame_sended)
        return possiblecallback

    # 确认单个callback
    def confirm_callback(self, possible_callback, recv_msg):
        # 判断报文内容
        # 反馈报文 -1.成功  -2.失败 -1 表示均不符合
        # 心跳报文 -1.错误报警报文 2.电机状态报文
        self.logger.debug("recv_msg.data is")
        self.logger.debug(recv_msg)
        self.logger.debug(recv_msg.data)
        self.logger.debug("possible callback is")
        self.logger.debug(possible_callback)
        if (recv_msg.data == possible_callback["True"].data):
            return True
        elif (recv_msg.data == possible_callback["False"].data):
            return False
        else:
            return -1

    # 下面是设置函数
    """
        -enable(id_list)
        -disable(id_list)
        -set_mode(self,id_list,data_dict) # 2F --速度 3F --位置  8F --力矩
        
        -set_posmode_absolutemode(id_list)
        -set_posmode_relativemode(id_list)
        -set_pos_acctime(id_list,acctime_dict)   100ms/单位
        -set_pos_trapvel(id_list,trapvel_dict)   rad/s 最大速度
        
        -set_vel(id_list,vel_dict)  rad/s
        -set_vel_acctime(id_list, acctime_dict)  100ms/单位
        -set_vel_stop(id_list)
    """
    # 电机使能
    # 要求能够向多个电机同时发送
    def enable(self,id_list):
        addr = [0x00,0x10]
        data_dict = {}
        for member in id_list:
            data_dict[member] = 0x1F
        return self.send_and_confirm(id_list,addr,data_dict)

    # 电机失能
    # 要求能够向多个电机同时发送
    def disable(self,id_list):
        addr = [0x00,0x10]
        data_dict = {}
        for member in id_list:
            data_dict[member] = 0x0F
        return self.send_and_confirm(id_list,addr,data_dict)

    # 设置电机模式
    # 2F --速度 3F --位置  8F --力矩
    def set_mode(self,id_list,data_dict):
        addr = [0x00, 0x19]
        return self.send_and_confirm(id_list, addr, data_dict)

    # 设置位置模式--绝对模式
    def set_posmode_absolutemode(self,id_list):
        addr = [0x00, 0x17]
        data_dict = {}
        for member in id_list:
            data_dict[member] = 0x4F
        return self.send_and_confirm(id_list,addr,data_dict)

    # 设置位置模式--相对模式
    def set_posmode_relativemode(self,id_list):
        addr = [0x00, 0x17]
        data_dict = {}
        for member in id_list:
            data_dict[member] = 0x5F
        return self.send_and_confirm(id_list,addr,data_dict)

    # 设置位置模式下--电机位置
    def set_pos(self,id_list,pos_dict):
        addr = [0x00, 0x16]
        data_dict = pos_dict
        return self.send_and_confirm(id_list, addr, data_dict)

    # 设置位置模式--加减速时间                       #单位是100毫秒
    def set_pos_acctime(self,id_list,acctime_dict):
        addr = [0x00, 0x12]
        data_dict = acctime_dict
        return self.send_and_confirm(id_list, addr, data_dict)

    # 设置位置模式--梯形速度                         #单位是rad/s
    def set_pos_trapvel(self,id_list,trapvel_dict):
        addr = [0x00, 0x14]
        data_dict = {}
        for member in id_list:
            data_dict[member] = int(trapvel_dict[member]*22.64331)  #换算单位,最大限速1000RPM,即104.666
        return self.send_and_confirm(id_list, addr, data_dict)

    # 设置电机速度                                  #速度的单位是 rad/s
    def set_vel(self,id_list,vel_dict):
        addr = [0x00, 0x11]
        data_dict = {}
        for member in id_list :
            data_dict[member] = int(vel_dict[member]*26.08949)  #换算单位  ecode_number/(rad/s)
        return self.send_and_confirm(id_list, addr, data_dict)


    # 设置速度模式下--加减速时间                   #单位是100毫秒,即几个100毫秒
    def set_vel_acctime(self, id_list, acctime_dict):
        addr = [0x00, 0x13]
        id = id_list
        data_dict = {}
        for member in id:
            data_dict[member] = acctime_dict[member]+2560
            """
            通过+2560完成下列操作
            msg.data[6] = 0x0A
            msg.data[4] = 0x00
            msg.data[5] = 0x00
            """
        return self.send_and_confirm(id_list, addr, data_dict)


    def set_vel_stop(self,id_list):
        addr = [0x00, 0x10]
        data_dict = {}
        for member in id_list:
            data_dict[member] = 0x0F
        return self.send_and_confirm(id_list, addr, data_dict)

    #读取消息并跟新电机状态
    def read_status(self,recv_msg):
        self.logger.debug("in the read status")
        self.logger.debug(recv_msg)
        if (recv_msg.data == self.statuscallback["Err_02"].data):
            print(recv_msg.arbitration_id,"号 驱动器过流！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x02
            return -1
        elif (recv_msg.data == self.statuscallback["Err_04"].data):
            print(recv_msg.arbitration_id,"号 驱动器过压！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x04
            return -1
        elif (recv_msg.data == self.statuscallback["Err_08"].data):
            print(recv_msg.arbitration_id,"号 驱动器编码器故障！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x08
            return -1
        elif (recv_msg.data == self.statuscallback["Err_10"].data):
            print(recv_msg.arbitration_id,"号 驱动器过热！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x10
            return -1
        elif (recv_msg.data == self.statuscallback["Err_20"].data):
            print(recv_msg.arbitration_id,"号 驱动器欠压！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x20
            return -1
        elif (recv_msg.data == self.statuscallback["Err_40"].data):
            print(recv_msg.arbitration_id,"号 驱动器过载！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x40
            return -1
        elif (recv_msg.data == self.statuscallback["Err_01"].data):
            print(recv_msg.arbitration_id,"号 驱动器运行状态！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x01
            return -1
        elif (recv_msg.data[0:4] == self.statuscallback["Stat_Pos"].data[0:4]):
            posbytes = recv_msg.data[4:8]
            pos = struct.unpack(">i",posbytes)[0]
            self.status[recv_msg.arbitration_id]["Pos"] = pos  #反馈位置
            return 2
        elif (recv_msg.data[0:4] == self.statuscallback["Stat_Vel_I"].data[0:4]):
            Vel = struct.unpack(">h",recv_msg.data[6:8])[0]
            I =   struct.unpack(">h",recv_msg.data[4:6])[0]
            self.status[recv_msg.arbitration_id]["Vel"] = Vel/8192*3000*(2*3.14/60) #单位为rad/s
            self.status[recv_msg.arbitration_id]["I"] = I/100   #单位为安培 A
            return 3


"""test"""
"""
    -enable(id_list)
    -disable(id_list)
    -set_mode(self,id_list,data_dict) # 2F --速度 3F --位置  8F --力矩

    -set_posmode_absolutemode(id_list)
    -set_posmode_relativemode(id_list)
    -set_pos_acctime(id_list,acctime_dict)   100ms/单位
    -set_pos_trapvel(id_list,trapvel_dict)   rad/s 最大速度

    -set_vel(id_list,vel_dict)  rad/s
    -set_vel_acctime(id_list, acctime_dict)  100ms/单位
    -set_vel_stop(id_list)
    
    -read_status(recv_msg)
"""

def test_send():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    msg = Message(extended_id=False)
    msg.arbitration_id = 0x01
    msg.dlc = 8
    msg.data = [0x00, 0xfa, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0f]
    bus.send(msg)
    bus.logger.debug("send finished!")
    bus.close_bus()
    bus.logger.debug("\n\n")

def test_recv():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    msg = Message(extended_id=False)
    msg.arbitration_id = 0x01
    msg.dlc = 8
    msg.data = [0x00, 0xfa, 0x00, 0x10, 0x00, 0x00, 0x00, 0x0f]
    bus.send(msg)
    recv_msg = bus.recv()
    bus.logger.debug("test_recv收到的消息是")
    bus.logger.debug(recv_msg)
    bus.close_bus()
    bus.logger.debug("\n")

def test_enable():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.enable([1])
    bus.close_bus()

def test_disable():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.disable([1])
    bus.close_bus()

def test_setmode():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.close_bus()

def test_set_absolte_posmode():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.logger.debug(bus.set_posmode_absolutemode([1]))
    bus.close_bus()

def test_set_relative_posmode():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.logger.debug(bus.set_posmode_relativemode([1]))
    bus.close_bus()

def test_relative_pos():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.enable([1])
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.logger.debug(bus.set_posmode_relativemode([1]))
    bus.logger.debug('\n\n')
    bus.set_pos([1],{1:10000})
    bus.close_bus()

def test_absolute_pos():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.enable([1])
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.logger.debug(bus.set_posmode_absolutemode([1]))
    bus.logger.debug('\n\n')
    bus.set_pos([1],{1:10000})
    bus.close_bus()

def test_set_vel():
    bus = ZL_motor_control('com20', 115200, 115200, {1})
    bus.open_bus()
    bus.logger.debug(bus.bus.ser)
    bus.bus.ser.setDTR(False)
    bus.bus.ser.flush()
    bus.enable([1])
    bus.logger.debug(bus.set_mode([1], {1: 0x2f}))
    bus.logger.debug('\n\n')
    starttime = time.time()
    while(True):
        bus.set_vel([1], {1:-5})
        timepass = time.time() - starttime
        if timepass > 10:
            bus.set_vel_stop([1])
            bus.logger.debug(bus.status)
            break
    bus.close_bus()

def test_set_posacctime():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.logger.debug(bus.set_mode([1],{1:0x3f}))
    bus.logger.debug(bus.set_posmode_absolutemode([1]))
    bus.set_pos_acctime([1],{1:20})
    bus.close_bus()

def test_set_vel_acctime():
    bus = ZL_motor_control('com20', 460800, 460800, {1})
    bus.open_bus()
    bus.bus.ser.setDTR(False)
    bus.logger.debug(bus.set_mode([1], {1: 0x2f}))
    bus.logger.debug(bus.set_pos_acctime([1], {1: 20}))
    bus.close_bus()


if __name__=="__main__":
    #test_send()
    #test_recv()
    #test_enable()
    #test_disable()
    #test_setmode()
    #test_relative_pos()
    #test_absolute_pos()
    #test_set_vel()
    #test_set_posacctime()