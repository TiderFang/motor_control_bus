#-*-coding:UTF-8-*-
try:
    import tkinter as tk  # for python 3
except:
    import Tkinter as tk  # for python 2
import pygubu
import os
import sys
sys.path.append("..")

import rospy
from scripts.car_control import *
from scripts.ZL_motor_control import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

try:
    import _thread
except:
    import thread as _thread

class Application:
    def __init__(self):

        #1: Create a builder
        self.builder = builder = pygubu.Builder()

        #2: Load an ui file
        builder.add_from_file('control_pad.ui')

        #3: Create the widget using a master as parent
        self.runwindow = builder.get_object('runapp')
        self.configwindow = builder.get_object('configapp',self.runwindow)
        self.configwindow.state("withdraw")
  
            
        #4: Connect to Delete event
        self.runwindow.protocol("WM_DELETE_WINDOW", self.quit)
        self.configwindow.protocol("WM_DELETE_WINDOW",self.quit)
        
        #5: Preserve the master
        self.master = self.runwindow
        
        #6: Connect Callbacks
        self.builder.connect_callbacks(self)
        
        #7: declaration of rosnode and bus
        self.controlnode = ros_wheel()
       
        #8: start the cmd_puber thread
        _thread.start_new_thread(self.cmd_thread,())
        
    def quit(self, event=None):
        self.runwindow.quit()
        self.configwindow.quit()
    
    def run(self):
        #  self.runwindow.mainloop()
        self.runwindow.mainloop()
        #self.configwindow.mainloop()
        
    def disable_object(self,name_list):
        for member in name_list:
            target = self.builder.get_object(member)
            target['state'] = 'disable'
            
    def enable_object(self,name_list):
        for member in name_list:
            target = self.builder.get_object(member)
            target['state'] = 'normal'
    
    
    def cmd_thread(self):
        speed_v_textbox = self.builder.get_object("speed_v_textbox")
        speed_w_textbox = self.builder.get_object("speed_w_textbox")
        go_up_button = self.builder.get_object("go_up_button")
        go_left_button = self.builder.get_object("go_left_button")
        go_left_up_button = self.builder.get_object("go_left_up_button")
        go_left_back_button = self.builder.get_object("go_left_back_button")
        stop_button = self.builder.get_object("stop_button")
        go_back_button = self.builder.get_object("go_back_button")
        go_right_up_button = self.builder.get_object("go_right_up_button")
        go_right_button = self.builder.get_object("go_right_button")
        go_right_back_button = self.builder.get_object("go_right_back_button")
        enable_button = self.builder.get_object("enable_button")
        normal = enable_button['state']
        r = rospy.Rate(10)
        rospy.loginfo("cmd_thread start")
        actual_v = self.builder.get_object("actual_speed_v_show")
        actual_w = self.builder.get_object("actual_speed_w_show")
        while(True):
            v = 0
            w = 0
            actual_v.configure(text=self.controlnode.state['v'])
            actual_w.configure(text=self.controlnode.state['w'])
            if (stop_button['state'] == normal):
                if (go_up_button['state'] == normal):
                    try:
                        v = float(speed_v_textbox.get())
                    except:
                        print("failed")
                        speed_v_textbox.configure(text=0)
                        v = 0
                elif (go_right_up_button['state'] == normal):
                    try:
                        v = float(speed_v_textbox.get())
                        w = float(speed_w_textbox.get())
                    except:
                        speed_v_textbox.set(0)
                        speed_w_textbox.set(0)
                        v = 0
                        w = 0
                elif (go_left_up_button['state'] == normal):
                    try:
                        v = float(speed_v_textbox.get())
                        w = -float(speed_w_textbox.get())
                    except:
                        speed_v_textbox['text']=0
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                elif (go_right_button['state'] == normal):
                    try:
                        v = 0
                        w = float(speed_w_textbox.get())
                    except:
                        speed_v_textbox['text']=0
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                elif (go_left_button['state'] == normal):
                    try:
                        v = 0
                        w = -float(speed_w_textbox.get())
                    except:
                        speed_v_textbox['text']=0
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                elif (go_right_back_button['state'] == normal):
                    try:
                        v = -float(speed_v_textbox.get())
                        w = -float(speed_w_textbox.get())
                    except:
                        speed_v_textbox['text']=0
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                elif (go_back_button['state'] == normal):
                    try:
                        v = -float(speed_v_textbox.get())
                        w = 0
                    except:
                        speed_v_textbox['text']=0
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                elif (go_left_back_button['state']== normal):
                    try:
                        v = -float(speed_v_textbox.get())
                        w = float(speed_w_textbox.get())
                    except:
                        speed_v_textbox.set(0)
                        speed_w_textbox['text']=0
                        v = 0
                        w = 0
                msg = Twist()
                msg.linear.x = v
                msg.angular.z = w
                print(msg)
                self.controlnode.cmd_puber.publish(msg)
            r.sleep()
                #print("publish a twist message")
    
    def on_run_mode_button(self):
        #  change the windows to run window
        
        self.configwindow.state("withdraw")
        self.runwindow.state("normal")
        
    def on_config_mode_button(self):
        # change the window to config window
        
        self.runwindow.state("withdraw")
        self.configwindow.state("normal")
        
    def on_enable_button(self):
        # if button state of control is disable, the button name is enable
        # if button state of control is enable , the button name is disable

        enable_button = self.builder.get_object("enable_button")
        config_button = self.builder.get_object("config_mode_button")
        stop_button = self.builder.get_object("stop_button")

        name_list =["speed_v_textbox","speed_w_textbox","go_up_button","go_left_button", 
                    "go_left_up_button","go_left_back_button","stop_button","go_back_button", 
                    "go_right_up_button","go_right_button","go_right_back_button","stop_button"] 
                    
        speed_v_textbox = self.builder.get_object("speed_v_textbox")
        
        if enable_button['text']=="disable hand control":
            self.disable_object(name_list)
            enable_button.configure(text="enable hand control")
            config_button['state']='normal'
            
        elif enable_button['text']=="enable hand control":
            self.enable_object(name_list)
            self.disable_object(['stop_button'])
            enable_button['text']="disable hand control"
            config_button['state']='disable'
    
    def set_control_button(self,button_name,button_string):
        go_button = self.builder.get_object(button_name)
        stop_button = self.builder.get_object("stop_button")
        
        name_list =["speed_v_textbox","speed_w_textbox","go_up_button","go_left_button","go_left_up_button",
        "go_left_back_button","stop_button","go_back_button","go_right_up_button",
        "go_right_button","go_right_back_button","enable_button"]
        
        if go_button['text']== button_string:
            self.disable_object(name_list)
            go_button['text']= button_string + ' on'
            go_button['state']='normal'
            stop_button['state']='normal'
            
        elif go_button['text']==button_string+' on':
            self.enable_object(name_list)
            stop_button['state']='disable'
            go_button['text']= button_string
            
    def on_go_up_button(self):
        self.set_control_button("go_up_button",'forward')

    
    def on_go_right_up_button(self):
        self.set_control_button("go_right_up_button","right_up")

    
    def on_go_left_up_button(self):
        self.set_control_button("go_left_up_button","left_up")


    def on_go_right_button(self):
        self.set_control_button("go_right_button","go right")
        
            
    def on_go_left_button(self):
        self.set_control_button("go_left_button","go left")
        
    
    def on_go_right_back_button(self):
        self.set_control_button("go_right_back_button","right_back")
        
            
    def on_go_back_button(self):
        self.set_control_button("go_back_button","back")
        
    def on_go_left_back_button(self):
        self.set_control_button("go_left_back_button","left_back")
    
    def on_stop_button(self):
    
        name_list = \
        ["speed_v_textbox","speed_w_textbox","go_up_button","go_left_button","go_left_up_button",
        "go_left_back_button","stop_button","go_back_button","go_right_up_button",
        "go_right_button","go_right_back_button","enable_button"]
        
        button_name = \
        ["go_up_button","go_left_button","go_left_up_button","go_left_back_button",
        "stop_button","go_back_button","go_right_up_button","go_right_button",
        "go_right_back_button","enable_button"]
        
        button_string_list = \
        ["forward","go left","left_up","left_back","stop","back","right_up",
        "go right","right_back","disable hand control"]
        
        for member in button_name:
            index = button_name.index(member)
            button = self.builder.get_object(member)
            button['text']=button_string_list[index]
            
        self.enable_object(name_list)
        self.disable_object(['stop_button'])
        
def odom_callback(msg,state):
    state['v'] = msg.twist.twist.linear.x
    state['w'] = msg.twist.twist.angular.z
    
class ros_wheel(object):
    def __init__(self):
    
        rospy.init_node("control_pad")
        self.state ={'v':0,'w':0}
        self.cmd_puber = rospy.Publisher("/cmd_vel",Twist,queue_size = 10)
        self.odom_suber = rospy.Subscriber("/odom",Odometry,odom_callback,(self.state))
        
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
            
        bus_channel = rospy.get_param("~bus_channel")
    	bus_baudrate = rospy.get_param("~bus_baudrate")
        bus_bitrate = rospy.get_param("~bus_bitrate")
        bus_type = rospy.get_param("~bus_type")
        bus_id_list = rospy.get_param("~bus_id_list")
        bus = ZL_motor_control(bus_channel,bus_bitrate,bus_baudrate,bus_id_list,bus_type)
        
if __name__ == '__main__':
    #root = tk.Tk()
    app = Application()
    app.run()

