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
"""

from __future__ import print_function
import struct
import time
from motor_control_bus import MotorControlBus
import copy
import logging
import serial


class amps_motor_control(MotorControlBus):

    def __init__(self, channel, baudrate):
        self.channel = channel
        self.baudrate = baudrate
        self.status = {}


    def open_bus(self):
        try:
            self.bus = serial.Serial(channel, baudrate, timeout=0.3)
            # self.bus.open()
        except:
            print("bus open failed! can't open serial port:", self.channel)
            return False
        finally:
            print("serial port %s openned!" % self.channel)

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

    def send(self, msg):
        try:
            self.bus.write(msg)
            return True
        except:
            print("send failure!")
            return False

    def recv(self, timeout = 0.1):
        # time.sleep(0.1)
        start = time.time()
        msg = bytes()
        while True:
            msg = msg + self.bus.read(10 - len(msg))
            if (time.time() - start) > timeout:
                print("read failed!")
                return None
            checksum = 0
            if len(msg) == 10:
                print(msg)
                for i in range(len(msg) - 1):
                    checksum = checksum + msg[i]
                print(checksum & 0xff)
                print(msg[9])
                if (checksum & 0xff) == msg[9]:
                    print("read a message!")
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
        for id in id_list:
            msg = self.ship_frame(id, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            recv_msg[id] = self.recv()
            if recv_msg[id][0] == id and recv_msg[id][2] == addr[0] and recv_msg[id][3] == addr[1]:
                result = result and (recv_msg[id] == 0x62)
        return result

    def disable(self, id_list):
        cmd = 0x52
        addr = [0x70, 0x19]
        error = 0x00
        data = 0x06
        recv_msg = {}
        result = True
        for id in id_list:
            msg = self.ship_frame(id, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
            recv_msg[id] = self.recv()
            if recv_msg[id][0] == id and recv_msg[id][2] == addr[0] and recv_msg[id][3] == addr[1]:
                result = result and (recv_msg[id] == 0x62)
        return result

    def read_status(self, id_list, timeout=0.3):
        cmd = 0xA0
        addr = [0x70, 0x75]
        error = 0x00
        data = 0x00
        for id in id_list:
            msg = self.ship_frame(id, cmd, addr, error, data)
            self.only_send_not_confirm(msg)
        start = time.time()
        id_cout = 0
        while(True):
            if (time.time() - start) > timeout:
                return False
            recv_msg = self.recv()
            #print("update_status",recv_msg)
            vel = {}
            if recv_msg[1] == 0xA2 and recv_msg[2:4] == bytearray(addr):
                data = recv_msg[7:9]
                vel[recv_msg[0]] = struct.unpack(">h", data)[0]
                id_cout = id_cout+1
                if id_cout == len(id_list):
                    #print("vel status is ",vel)
                    for id in id_list:
                        self.status[id] = vel[id]
                    return vel

    def set_mode(self, id_list, mode_dict):
        # 3 带加减速控制的速度模式
        # -3 立即速度模式
        # 4 力矩模式
        # 1 位置模式
        # 6 原点模式
        cmd = 0x51
        addr = [0x70, 0x17]
        error = 0x00
        recv_msg = {}
        result = True
        for member in id_list:
            msg = self.ship_frame(member, cmd, addr, error, mode_dict[member])
            self.only_send_not_confirm(msg)
            if msg is not None:
                result = (result and (recv_msg[0] == msg[0] and recv_msg[1] == 0x62 and
                                      recv_msg[2] == addr[0] and recv_msg[3] == addr[1]))
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
            recv_msg = self.recv()
            #print("set_vel:",recv_msg)



def test_enable_disable(bus):
    bus.enable([1])
    input("Press any key!")
    bus.disable([1])


def test_get_vel(bus):
    print(bus.read_status([1]))
    input("Press any key!")


def test_set_vel(bus,vel):
    bus.enable([1])
    while True:
        speed = input("input speed!")
        if speed != '' and speed != 's':
            bus.set_vel([1],{1:float(speed)})
            bus.read_status([1])
            # input("press enter!")
        elif speed == "s":
            bus.read_status([1])


if __name__ == "__main__":
    channel = 'COM27'
    baudrate = 115200
    id_list = [1]
    bus = amps_motor_control(channel, baudrate)
    input("open port,press enter!")
    bus.open_bus()
    # ------------------------------------------
    test_enable_disable(bus)
    input("test set_vel")
    # test_get_vel(bus)
    test_set_vel(bus,1)
    # ------------------------------------------
    bus.close_bus()
