# /usr/bin/env python
#-*_coding:UTF-8-*-

#@brief
"""
模块目标：
    -提供控制中菱电机、步进电机以及舵机的抽象类。
    -bus是指传输路径；bus可以由多个电机共享。
"""
import abc

class MotorControlBus(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def __init__(self,channel,baudrate):
        self.channel = channel
        self.baudrate = channel
        return True

#"""打开bus"""
    @abc.abstractmethod
    def open_bus(self):
        return True

#"""关闭bus"""
    @abc.abstractmethod
    def close_bus(self):
        return False

#"""电机使能"""
    @abc.abstractmethod
    def enable(self):
        pass

#"""电机失能"""
    @abc.abstractmethod
    def disable(self):
        pass

#"""设置电机运动模式"""
    @abc.abstractmethod
    def set_mode(self,mode):
        return True

#"""设置电机PDI参数"""
    @abc.abstractmethod
    def set_PID(self,P,I,D):
        return True

#"""设置电机最大加速度"""
    @abc.abstractmethod
    def set_Acc(self,acc):
        return True

#"""设置电机加速时间"""
    @abc.abstractmethod
    def set_acctime(self,time):
        return True

#"""设置电机最大转速度"""
    @abc.abstractmethod
    def set_maxvel(self,maxvel):
        return True
#"""装填帧"""
    @abc.abstractmethod
    def ship_frame(data):
        return True

#"""发送帧"""
    @abc.abstractmethod
    def send_frame(frame):
        return True

#"""计算反馈帧"""
    @abc.abstractmethod
    def cal_callbackframe(frame_sended):
        return True

#"""确认反馈帧"""
    @abc.abstractmethod
    def confirm_callback(frame_sended):
        return True

#"""读取电机状态"""


