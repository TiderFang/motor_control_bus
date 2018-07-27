# /usr/bin/env python
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


class ZL_motor_control(MotorControlBus):

    #初始化
    """
        channel：表示串口号
        bitrate:表示波特率或者Can速率
        baudrate:表示波特率
        bustype:表示can bus的类型
    """
    def __init__(self, channel, bitrate, baudrate, id_list, bustype = 'serial'):
        self.channel = channel
        self.bitrate = bitrate
        self.bustype = bustype
        self.timout = 0.3
        self.id = id_list
        #打开bus
        self.status = {}
        self.statuscallback = {}
        for member in self.id:
            self.status[member] = {"Pos":0, "Vel":0, "I":0, "Err":0}
        self.open_bus()
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

    # 打开bus
    def open_bus(self):
        try:
            self.bus = can.interface.Bus(bustype=self.bustype, channel=self.channel, biterate=self.bitrate)
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

    # 装填帧
    # 输入数据帧，返回Message数据
    """
        id:表示从站id,单个id
        addr:表示寄存器地址，由地址高地位表示addr1,addr2]
        value:表示目标数据，32位数据。
    """
    def ship_frame(self, id, addr, value):
        valuebytes = struct.pack(">i",value)
        data = bytearray([0x00,0xFA,addr[0],addr[1]])
        data = data + valuebytes
        frame = Message(data=data)
        frame.arbitration_id = id
        frame.timestamp = time.time()
        frame.dlc = 8
        return frame

    # 发送帧
    # 发送Can.Message类型的数据
    def send_frame(self,frame):
        try:
            self.bus.send(frame,0.2)
            return True
        except ValueError as e:
            print(e)
            return False
        except:
            print("send failed!")
            return False

    #"""电机使能"""
    #要求能够向多个电机同时发送
    def enable(self,id_list):
        addr = [0x00,0x10]
        data = 0x1F
        possible_callback = {}
        callback_result = {}
        id = id_list
        for member in id :
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] == False or callback_result[member] == -1:
                return [False, callback_result]
        return True

    #"""电机失能"""
    #要求能够向多个电机同时发送
    def disable(self,id_list):
        addr = [0x00,0x10]
        data = 0x0F
        possible_callback = {}
        callback_result = {}
        id = id_list
        for member in id :
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    #”“”设置电机模式
    def set_mode(self,id_list,mode_list):
        addr = [0x00, 0x19]
        possible_callback = {}
        callback_result = {}
        id = id_list
        mode = mode_list
        for member in id :
            data = mode[id.index(member)]
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    #"""设置位置模式--绝对模式
    def set_posmode_absolutemode(self,id_list):
        addr = [0x00, 0x17]
        possible_callback = {}
        callback_result = {}
        id = id_list
        data = 0x4F
        for member in id :
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    #"""设置位置模式--相对模式
    def set_posmode_torelativemode(self,id_list):
        addr = [0x00, 0x17]
        possible_callback = {}
        callback_result = {}
        id = id_list
        data = 0x5F
        for member in id :
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    #"""设置位置模式下--电机位置
    def set_pos(self,id_list,pos_list):
        addr = [0x00, 0x16]
        possible_callback = {}
        callback_result = {}
        id = id_list
        pos = pos_list
        for member in id :
            data = pos[id.index(member)]
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    # """设置位置模式--加减速时间                       #单位是100毫秒
    def set_pos_acctime(self,id_list,acctime_list):
        addr = [0x00, 0x12]
        possible_callback = {}
        callback_result = {}
        id = id_list
        acctime = acctime_list
        for member in id:
            data = acctime[id.index(member)]
            msg = self.ship_frame(member, addr, data)
            msg.data[6] = 0x0A
            msg.data[4] = 0x00
            msg.data[5] = 0x00
            try:
                self.bus.send(msg, timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member] = self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback, callback_result)
        for member in id:
            if callback_result[member] == False:
                return [False, callback_result]
        return True

    # """设置位置模式--梯形速度                         #单位是rad/s
    def set_pos_trapvel(self,id_list,trapvel_list):
        addr = [0x00, 0x14]
        possible_callback = {}
        callback_result = {}
        id = id_list
        trapvel = trapvel_list
        for member in id:
            data = int(trapvel[id.index(member)]*22.64331)  #换算单位,最大限速1000RPM,即104.666
            msg = self.ship_frame(member, addr, data)
            try:
                self.bus.send(msg, timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member] = self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback, callback_result)
        for member in id:
            if callback_result[member] == False:
                return [False, callback_result]
        return True

    #”“”设置电机速度
    def set_vel(self,id_list,vel_list):   #速度的单位是 rad/s
        addr = [0x00, 0x11]
        possible_callback = {}
        callback_result = {}
        id = id_list
        vel = vel_list
        for member in id :
            data = int(vel[id.index(member)]*26.08949)  #换算单位  ecode_number/(rad/s)
            msg = self.ship_frame(member,addr,data)
            try:
                self.bus.send(msg,timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member]=self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback,callback_result)
        for member in id:
            if callback_result[member] ==False:
                return [False,callback_result]
        return True

    # “”“设置加速度模式下--加减速时间                   #单位是100毫秒
    def set_vel_acctime(self,id_list,acctime_list):   #单位是毫秒
        addr = [0x00, 0x13]
        possible_callback = {}
        callback_result = {}
        id = id_list
        acctime = acctime_list
        for member in id:
            data = acctime[id.index(member)]
            msg = self.ship_frame(member, addr, data)
            msg.data[6] = 0x0A
            msg.data[4] = 0x00
            msg.data[5] = 0x00
            try:
                self.bus.send(msg, timeout=0.3)
            except:
                print("send failed!")
                return False
            finally:
                possible_callback[member] = self.cal_callbackframe(msg)
                callback_result[member] = -1
        self.check_callback(possible_callback, callback_result)
        for member in id:
            if callback_result[member] == False:
                return [False, callback_result]
        return True

    # 检查反馈报文
    # 设置检测时间变量
    # 将报文反馈结果存储
    def check_callback(self,possible_callback,callback_result):
        starttime = time.time()
        while(True):
            #计算时间长度
            timepass = time.time() - starttime
            #时间长度大于设置时间，判定为失败
            if(timepass >= self.timout):
                return False
            # 在timeout时间内检测到反馈报文
            # 接受报文
            try:
                recv_msg = self.bus.recv(timeout = self.timout)
            except can.CanError as e:
                print(e)
                return False
            # 报文ID
            callback_id = copy.deepcopy(id)
            if recv_msg.arbitration_id in callback_id:
                member = recv_msg.arbitration_id
                result = self.confirm_callback(possible_callback[member],recv_msg)
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
    #["True","False"]
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
        if (recv_msg == possible_callback["True"]):
            return True
        elif (recv_msg == possible_callback["False"]):
            return False
        else:
            return -1

    def read_status(self,recv_msg):
        if (recv_msg == self.statuscallback["Err_02"]):
            print(recv_msg.arbitration_id,"号 驱动器过流！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x02
            return -1
        elif (recv_msg == self.statuscallback["Err_04"]):
            print(recv_msg.arbitration_id,"号 驱动器过压！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x04
            return -1
        elif (recv_msg == self.statuscallback["Err_08"]):
            print(recv_msg.arbitration_id,"号 驱动器编码器故障！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x08
            return -1
        elif (recv_msg == self.statuscallback["Err_10"]):
            print(recv_msg.arbitration_id,"号 驱动器过热！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x10
            return -1
        elif (recv_msg == self.statuscallback["Err_20"]):
            print(recv_msg.arbitration_id,"号 驱动器欠压！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x20
            return -1
        elif (recv_msg == self.statuscallback["Err_40"]):
            print(recv_msg.arbitration_id,"号 驱动器过载！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x40
            return -1
        elif (recv_msg == self.statuscallback["Err_01"]):
            print(recv_msg.arbitration_id,"号 驱动器运行状态！")
            self.status[recv_msg.arbitration_id]["Err"] = 0x01
            return -1
        elif (recv_msg.data[0:4] == self.statuscallback["Stat_Pos"].data[0:4]):
            posbytes = recv_msg.data[5:8]
            pos = struct.unpack(">i",posbytes)[0]
            self.status[recv_msg.arbitration_id]["Pos"] = pos  #反馈位置
            return 2
        elif (recv_msg.data[0:4] == self.statuscallback["Stat_Vel_I"].data[0:4]):
            Vel = struct.unpack(">f",recv_msg.data[5:6])[0]
            I = struct.unpack(">f",recv_msg.data[7:8])[0]
            self.status[recv_msg.arbitration_id]["Vel"] = Vel/8192*3000*(2*3.14/60) #单位为rad/s
            self.status[recv_msg.arbitration_id]["I"] = I/100   #单位为安培 A
            return 3
