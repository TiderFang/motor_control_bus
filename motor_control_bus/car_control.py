# /usr/bin/env python
# -*- coding:utf-8 -*-

#@ brief introduction
"""
本段程序描述了移动机器人底盘的一些控制方法，包括使能、前进、设置底盘速度等
底盘电机的控制可以通过多种方式，例如：can modbus等
"""

from ZL_motor_control import ZL_motor_control
# class ZL_motor_control
import can
from math import *

# 外部设置bus，然后传递给car
class car(object):
    def __init__(self,wheel_diameter,wheel_distance,bus,id_list):
        self.bus = bus
        self.diameter = wheel_diameter
        self.distance = wheel_distance
        self.id_list = id_list
        self.odom = {'x':0,'y':0,'theta':0}
        self.isRunMode = False

    def enable(self):
        return self.bus.enable(self.id_list)

    def disable(self):
        return self.bus.disable(self.id_list)

    #根据车的速度，计算轮子的速度
    def set_car_vel(self,v,w):
        data_list = self.cal_wheel_vel(v,w)
        return self.bus.set_vel(self.id_list,data_list)

    # 获取车的速度和转速
    def get_car_status(self):
        w1 = self.bus.status[self.id_list[0]]["Vel"]
        w2 = self.bus.status[self.id_list[1]]["Vel"]
        w = (w1+w2)*self.diameter/2/self.diameter
        v = (w1-w2)*self.diameter/2
        return [v,w]

    # 计算轮子速度
    def cal_wheel_vel(self,v,w):
        w1 = 2*v/self.diameter - w*self.distance/self.diameter
        w2 = -(2*v/self.diameter + w*self.distance/self.diameter)
        return [w1,w2]

    #设置车辆odom信息
    def set_odom(self):
        dt = 0.1
        v,w = self.get_car_status()
        self.odom['x']= self.odom['x'] + v*dt*cos(self.odom['theta'])
        self.odom['y']= self.odom['y'] + v*dt*sin(self.odom['theta'])
        self.odom['theta'] = self.odom['theta'] + w*dt
        self.odom['v'] = v
        self.odom['w'] = w

    # 进入config mode,关闭bus
    def config_mode(self):
        self.bus.set_vel_stop(self.id_list)
        self.bus.disable()
        self.isRunMode = False
        return self.bus.close_bus()

    # 进入run_mode，就可以进行速度控制了
    def run_mode(self):
        self.bus.open_bus()
        self.bus.enable(self.id_list)
        # 设置运动模式--速度模式
        mode = {}
        for member in self.id_list:
            mode[member] = 0x2F
        self.bus.set_mode(self.id_list,mode)
        self.bus.enable(self.id_list)
        self.isRunMode = True

    # 跟新轮子信息以及车子信息
    def update_status(self):
        try:
            recv_msg = self.bus.recv(timeout=self.timout)
        except can.CanError as e:
            print(e)
            return False
        finally:
            self.bus.read_status(recv_msg)
            self.get_car_status()
