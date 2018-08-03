#!/usr/bin/env python
#-*-coding:UTF-8-*-

# @brief
"""
this file is used to test motor_control_bus module.
"""

from ZL_motor_control import *
import can
import test

# 打开串口的测试
def object_test():
    channel='com10'
    baudrate = 115200
    bitrate = 460800
    bustype = 'serial'
    motor = ZL_motor_control(channel,baudrate,bitrate,bustype)
    return motor

# 装帧的测试
def ship_frame_test(bus,id,addr,data):
    return bus.ship_frame(id,addr,data)

if __name__ == '__main__':
    bus = object_test()
    msg = can.Message(data=[0x00,0xFA,0x00,0x01,0x00,0x00,0x00,0x0F])
    msg.timestamp = 0
    msg.arbitration_id = 0
    id =0
    addr = [0x00,0x01]
    data = 15
    msg_test = ship_frame_test(bus,id,addr,data)
    msg_test.timestamp = 0
    print(msg.data)
    print(msg_test.data)
    assert(msg==msg_test)
    assert (bus.bus.send(msg))

