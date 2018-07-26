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



class ZL_motor_control(MotorControlBus):

    #初始化
    """
        channel：表示串口号
        bitrate:表示波特率或者Can速率
        baudrate:表示波特率
        bustype:表示can bus的类型
    """
    def __init__(self, channel, bitrate, baudrate, bustype = 'serial'):
        self.channel = channel
        self.bitrate = bitrate
        self.bustype = bustype
        #打开bus
        self.open_bus()

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
        id:表示从站id
        addr:表示寄存器地址，由地址高地位表示addr1,addr2]
        value:表示目标数据，32位数据。
    """
    def ship_frame(self, id, addr, value):
        valuebytes = struct.pack(">i",value)
        data = bytearray([0x00,0xFA,addr[0],addr[1]])
        data.append(valuebytes)
        frame = Message(data)
        frame.arbitration_id = id
        frame.timestamp = time.time()
        frame.dlc = 8
        return frame



