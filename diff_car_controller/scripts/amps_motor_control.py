#!/usr/bin/env python
# -*-coding:UTF-8-*-

# introduction
"""
用途：
    安普斯轮毂电机驱动器控制类；本类使用rs485总线控制安普斯电机驱动器。
    rs485总线经过usb转485模块与驱动器相连。
功能：
    1.提供bus对象，实现io读取发送；
    2.提供驱动器的基本控制函数，是电机基本功能调用的底层实现：
        1.使能与使能
        2.模式选择
        3.参数设置
        4.读取参数
        5.设置目标运动值（速度、力矩等）
区别：
    本程序与ZL_motor_control的区别在于：本程序使用的是usb转rs485总线。

    取消對python3的支持，僅僅支持python2
"""

from __future__ import print_function
import struct
import time
#from motor_control_bus import MotorControlBus
import copy
import logging
import serial
import can

class amps_motor_control(object):

    def __init__(self, channel, baudrate):
        self.channel = channel
        self.baudrate = baudrate
        self.status = {}
        self.bus = None


    def open_bus(self):
        try:
            self.bus = serial.Serial(self.channel, self.baudrate, timeout=0.3)
            print("serial port %s openned!" % self.channel)
            # self.bus.open()
            return True
        except:
            print("bus open failed! can't open serial port:", self.channel)
            return False
            

    def close_bus(self):
        self.bus.close()

    def ship_frame(self, idnum, cmd, addr, error, data):
        databyte = struct.pack(">i", data)
        idbyte = struct.pack(">B", idnum)
        cmdbyte = struct.pack(">B", cmd)
        errorbyte = struct.pack(">B", error)
        msg = idbyte + cmdbyte + bytearray(addr) + errorbyte + databyte
        checksum = 0
        for i in range(len(msg)):
            checksum = checksum + msg[i]
        checksum = checksum & 0xFF
        checksum = struct.pack(">B", checksum)
        msg = msg + checksum
        return msg

    def feedback_frame(self, idnum, cmd, addr, error, data):
        databyte = struct.pack(">i", data)
        idbyte = struct.pack(">B", idnum)
        cmdbyte = None
        if cmd != 0xA0:
            cmdbyte = struct.pack(">B", cmd + 0x10)
        errorbyte = struct.pack(">B", error)
        msg = idbyte + cmdbyte + bytearray(addr) + errorbyte + databyte
        checksum = 0
        for i in range(len(msg)):
            checksum = checksum + msg[i]
        checksum = checksum & 0xFF
        checksum = struct.pack(">B", checksum)
        msg = msg + checksum
        return msg
    
    def strtobytes(self,msg):
        byte_list = []
        for member in msg:
            byte_list.append(ord(member))
        return bytes(byte_list)

    def send(self, msg):
        try:
            #self.bus.write(bytes[0x00,0x00])
            self.bus.write(msg)
            return True
        except:
            print("send failure!")
            return False
        
    # only support for python2
    def recv(self, timeout = 0.1):
        # time.sleep(0.1)
        start = time.time()
        msg = bytes()
        while True:
            temp = self.bus.read(10 - len(msg))
            msg = msg + temp
            if (time.time() - start) > timeout:
                # print("read failed!")
                return None
            checksum = 0
            if len(msg) == 10:
                for i in range(len(msg) - 1):
                    # for python2
                    checksum = checksum + ord(msg[i])
                msg_checksum = ord(msg[9])
                if (checksum & 0xff) == msg_checksum:
                    #print("read a message!it's id is ",ord(msg[0]))
                    return msg
                else:
                    msg = msg[1:10]

    # send a command and confirm its acception
    def send_and_confirm(self, msg):
        result = True
        for member in msg:
            self.send(member)
            recv_msg = self.recv()
            if recv_msg[1] == msg[1] + 0x10:
                result = result and True
            else:
                result = False

    def only_send_not_confirm(self, msg):
        self.send(msg)

    def enable(self, id_list):
        cmd = 0x52
        addr = [0x70, 0x19]
        error = 0x00
        data = 0x0F
        recv_msg = {}
        result = True
        for member in id_list:
            msg = self.ship_frame(member, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            recv_msg[member] = self.recv()
            if recv_msg[member][0] == member and ord(recv_msg[member][2]) == addr[0] and ord(recv_msg[member][3]) == addr[1]:
                result = result and (ord(recv_msg[member][1]) == 0x62)
        return result

    def disable(self, id_list):
        cmd = 0x52
        addr = [0x70, 0x19]
        error = 0x00
        data = 0x06
        recv_msg = {}
        result = True
        for member in id_list:
            msg = self.ship_frame(member, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            recv_msg[member] = self.recv()
            if recv_msg[member][0] == member and ord(recv_msg[member][2]) == addr[0] and ord(recv_msg[member][3]) == addr[1]:
                result = result and (ord(recv_msg[member][1]) == 0x62)
        return result

	# in use
    def only_send_status(self,id_list):
        cmd = 0xA0
        addr = [0x70, 0x75]
        error = 0x00
        data = 0x00
        for idnum in id_list:
            #print("send status : ",idnum)
            msg = self.ship_frame(idnum, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            time.sleep(0.002)

    # in use
    def only_read_status(self, timeout=0.3):
        cmd = 0xA0
        addr = [0x70, 0x75]
        error = 0x00
        recv_msg = self.recv() 
        if recv_msg != None and ord(recv_msg[2]) == addr[0] and ord(recv_msg[3]) == addr[1]:
            data = recv_msg[7:9]
            self.status[ord(recv_msg[0])] = struct.unpack('>h', data)[0]
        else:
            return False
        
    # not in use
    def read_status(self, id_list, timeout=0.3):
        cmd = 0xA0
        addr = [0x70, 0x75]
        error = 0x00
        data = 0x00
        recv_msg = {}
        id_flag = copy.deepcopy(id_list)
        for idnum in id_list:
            msg = self.ship_frame(idnum, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            recv_msg[idnum] = self.recv()
        while len(id_flag) != 0:
            recv_msg = self.recv()
            if recv_msg != None and  ord(recv_msg[2]) == addr[0] and ord(recv_msg[3]) == addr[1]:
                data = recv_msg[7:9]
                self.status[ord(recv_msg[0])] = struct.unpack('>h', data)[0]
                id_flag.remove(ord(recv_msg[0]))
            
                

    def set_mode(self, id_list, mode_dict):
        # 3 带加减速控制的速度模式
        # -3 立即速度模式
        # 4 力矩模式
        # 1 位置模式
        # 6 原点模式
        cmd = 0x51
        addr = [0x70, 0x17]
        error = 0x00
        recv_msg = None
        result = True
        for member in id_list:
            msg = self.ship_frame(member, cmd, addr, error, mode_dict[member])
            self.only_send_not_confirm(msg)
            recv_msg = self.recv()
            #print("set_mode recv_msg ",recv_msg)
            if msg is not None:
                result = (result and (ord(recv_msg[0]) == msg[0] and ord(recv_msg[1]) == 0x62 and
                                      ord(recv_msg[2]) == addr[0] and ord(recv_msg[3]) == addr[1]))
        return result

    def set_vel(self, id_list, vel_dict):  # rad/s
        cmd = 0x52
        addr = [0x70, 0xB1]
        error = 0x00
        for member in id_list:
            data = int(vel_dict[member]*60/2/3.14)
            #print("begin to read for set_vel")
            msg = self.ship_frame(member, cmd, addr, error, data)
            self.send(msg)
            time.sleep(0.002)



def test_enable_disable(bus):
    bus.enable([1])
    input("Press any key!")
    bus.disable([1])


def test_get_vel(bus):
    print(bus.read_status([1]))
    input("Press any key!")


def test_set_vel(bus,id_list):
    bus.enable(id_list)
    vel_dict = {}
    while True:
        for idnum in id_list:
            while True:
                try:
                    speed = raw_input("idnum is %i, input target speed!\n" % idnum)
                except:
                    speed = input("idnum is %i, input target speed!\n" %idnum)
                if speed != '' and speed != 's':
                    vel_dict[idnum] = float(speed)
                    break
                    #bus.set_vel(id_list,{2:float(speed)})
                    #bus.read_status([2])
                    # input("press enter!")
                elif speed == "s":
                    bus.read_status(id_list)
        bus.set_vel(id_list,vel_dict)
        bus.read_status(id_list)


if __name__ == "__main__":
    channel = '/dev/ttyUSB0'
    baudrate = 115200
    id_list = [1,2]
    bus = amps_motor_control(channel, baudrate)
    input("open port,press enter!")
    bus.open_bus()
    # ------------------------------------------
    test_enable_disable(bus)
    input("test set_vel")
    # test_get_vel(bus)
    test_set_vel(bus,[1,2])
    # ------------------------------------------
    bus.close_bus()
