#!/usr/bin/env python
# -*- coding:utf-8 -*-

#@ brief introduction
"""
本段程序描述了移动机器人底盘的一些控制方法，包括使能、前进、设置底盘速度等
底盘电机的控制可以通过多种方式，例如：can modbus等
"""

from amps_motor_control import amps_motor_control
# class ZL_motor_control
from math import *
import time
import can

# 外部设置bus，然后传递给car
class car(object):
    def __init__(self,wheel_diameter,wheel_distance,bus,id_list):
        self.bus = bus
        self.diameter = wheel_diameter
        self.distance = wheel_distance
        self.id_list = id_list
        self.odom = {'x':0,'y':0,'theta':0,'v':0,'w':0}
        self.isRunMode = False
        self.isSending = False
        for member in id_list:
            self.bus.status[member] = 0

    def enable(self):
        return self.bus.enable(self.id_list)

    def disable(self):
        return self.bus.disable(self.id_list)

    #根据车的速度，计算轮子的速度
    def set_car_vel(self,v,w):
        data_dict = self.cal_wheel_vel(v,w)
        return self.bus.set_vel(self.id_list,data_dict)


    # 计算轮子速度
    # 1左 2右
    def cal_wheel_vel(self,v,w):
        w1 = 2*v/self.diameter - w*self.distance/self.diameter
        w2 = -2*v/self.diameter + w*self.distance/self.diameter
        return {self.id_list[0]:w1,self.id_list[1]:w2}
    
    # 获取车的速度和转速
    def get_car_status(self):
        #print(self.bus.status)
        w1 = self.bus.status[self.id_list[0]]
        w2 = self.bus.status[self.id_list[1]]
        w = (w1+w2)*self.diameter/2/self.diameter
        v = (w1-w2)*self.diameter/2
        return [v,w]
    
    #设置车辆odom信息
    def set_odom(self):
        dt = 0.05
        #print("set odom")
        v,w = self.get_car_status()
        self.odom['x']= self.odom['x'] + v*dt*cos(self.odom['theta'])
        self.odom['y']= self.odom['y'] + v*dt*sin(self.odom['theta'])
        self.odom['theta'] = self.odom['theta'] + w*dt
        self.odom['v'] = v
        self.odom['w'] = w

    # 进入config mode,关闭bus
    def config_mode(self):
        #self.bus.bus.ser.setDTR(False)
        self.bus.set_vel_stop(self.id_list)
        self.bus.disable(self.id_list)
        self.isRunMode = False
        self.bus.close_bus()

    # 进入run_mode，就可以进行速度控制了
    def run_mode(self):
        self.bus.open_bus()
        #self.bus.setDTR(False)
        # 设置运动模式--速度模式
        mode = {}
        for member in self.id_list:
            mode[member] = 0x3
        self.bus.set_mode(self.id_list,mode)
        self.bus.enable(self.id_list)
        self.isRunMode = True
        print("diff_car go into the run mode")

    # 跟新轮子信息以及车子信息
    def update_status(self):
        recv_msg = None
        try:
            #self.bus.read_status(self.id_list)
            self.bus.only_read_status()
            #print(self.bus.status)
        except:
            return False
        finally:
            #print("update car status")
            self.set_odom()
                
                
def test_set_car_vel(v,w,bus):
    bus = amps_motor_control(bus_channel, bus_baudrate)
    #bus.open_bus()
    diff_car = car(wheel_diameter, wheel_distance, bus, bus_id_list)
    diff_car.run_mode()
    start = time.time()
    while True:
        diff_car.set_car_vel(v,w)
        if (start-time.time()>2):
            break

def test_car_run_mode():
    bus_channel = "/dev/ttyUSB0"
    bus_bitrate = bus_baudrate = 115200
    bus_id_list = [1,2]
    bus_type='serial'
    wheel_diameter = 100
    wheel_distance = 100
    bus = amps_motor_control(bus_channel, bus_baudrate)
    #bus.open_bus()
    diff_car = car(wheel_diameter,wheel_distance,bus,bus_id_list)
    diff_car.run_mode()
    
def test_car_config_mode():
    bus_channel = "/dev/ttyUSB0"
    bus_bitrate = bus_baudrate = 115200
    bus_id_list = [1, 2]
    bus_type='serial'
    wheel_diameter = 100
    wheel_distance = 100
    bus = amps_motor_control(bus_channel, bus_baudrate)
    diff_car = car(wheel_diameter,wheel_distance,bus,bus_id_list)
    diff_car.config_mode()

if __name__ == '__main__':
    #test_car_config_mode()
    #test_car_run_mode()
    bus_channel = "/dev/ttyUSB0"
    bus_bitrate = bus_baudrate = 115200
    bus_id_list = [1,2]
    wheel_diameter = 100
    wheel_distance = 100
    bus = amps_motor_control(bus_channel, bus_baudrate)
    test_set_car_vel(0,0,bus)
    
    
