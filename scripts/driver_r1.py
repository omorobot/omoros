#!/usr/bin/env python
import sys

import rospy
import serial
import io
import numpy as np
import tf2
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry
from omoros.msg import R1MotorStatusLR, R1MotorStatus
from tf2.transformations import quaternion_about_axis
from tf2.broadcaster import TransformBroadcaster
from copy import copy, deepcopy
from sensor_msgs.msg import Joy

class Robot:
    ser = serial.Serial('/dev/ttyS0', 115200)
    ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                              newline = '\r',
                              line_buffering = True)
    Vl_max_mm_s = 500 #Maximum speed mm/s
    joyAxes = []
    joyButtons = []
    joyDeadband = 0.15
    exp = 0.3
    rcvCnt = 0
    isAutoMode = False
    #initialize data
    enc_L = 0
    enc_R = 0
    odo_L = 0
    odo_R = 0
    RPM_L = 0
    RPM_R = 0
    Vl = 0.0
    Vr = 0.0
    
    def __init__(self):
        print(self.ser.name)         # check which port was really used
        self.joyAxes = [0,0,0,0,0,0,0,0]
        self.joyButtons = [0,0,0,0,0,0,0,0]
        # Configure data output
        if self.ser.isOpen():
            print("Serial Open")
            self.resetODO()
            self.setREGI(0,'QENCOD')
            self.setREGI(1,'QODO')
            self.setREGI(2,'QRPM')
            self.setSPERI(20)
            self.setPEEN(1)
            
        self.reset_odometry()   
        rospy.init_node('omoros', anonymous=True)
        
        #self.br = TransformBroadcaster()
        # Subscriber
        rospy.Subscriber("joy", Joy, self.callbackJoy)
        # publisher
        self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
        self.pub_odometry = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pub_motor_status = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
        #self.pub_joints = rospy.Publisher("joint_state", JointState, queue_size=10)
        
        rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
        rospy.Timer(rospy.Duration(0.1), self.joytimer)
        rospy.Timer(rospy.Duration(0.01), self.serReader)
        
        while not rospy.is_shutdown():
            rate.sleep()
            
        self.ser.close()
            
    def callbackJoy(self, data):
        self.joyAxes = deepcopy(data.axes)
        # Read the most recent button state
        newJoyButtons = [0,0,0,0,0,0,0,0]
        newJoyButtons = deepcopy(data.buttons)
        # Check if button 1(B) is newly set
        if (newJoyButtons[1]==1) and (newJoyButtons[1]!=joyButtons[1]):
            if self.isAutoMode!= True:
                self.isAutoMode = True
            else:
                self.isAutoMode = False
        # Update button state
        self.joyButtons = deepcopy(newJoyButtons)

    def serReader(self, event):
        reader = self.ser_io.readline()
        if reader:
            packet = reader.split(",")
            try:
                header = packet[0].split("#")[1]
                if header.startswith('CVW'):
                    self.Vl = int(packet[1])
                    self.Vr = int(packet[2])

                elif header.startswith('QENCOD'):
                    self.enc_L = int(packet[1])
                    self.enc_R = int(packet[2])
                    self.pub_enc_l.publish(Float64(data=self.enc_L))
                    self.pub_enc_r.publish(Float64(data=self.enc_R))
                    #print('{:04d},{:04d}'.format(self.enc_L, self.enc_R))
                elif header.startswith('QODO'):
                    self.odo_L = int(packet[1])
                    self.odo_R = int(packet[2])
                elif header.startswith('QRPM'):
                    self.RPM_L = int(packet[1])
                    self.RPM_R = int(packet[2])
                    #print('{:04d},{:04d}'.format(self.RPM_L, self.RPM_R))
            except:
                print('Wrong packet')
                pass
                
            status_left = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                                        encoder = self.enc_L, RPM = self.RPM_L, ODO = self.odo_L)
            status_right = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                                        encoder = self.enc_R, RPM = self.RPM_R, ODO = self.odo_R)
            self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()), left=status_left, right=status_right))        
            
                
    def joytimer(self, event):
        if self.isAutoMode!= True:
            self.joy_v = self.joyAxes[1]
            self.joy_w = self.joyAxes[0]
            #print "Joy mode: {:.2f} {:.2f} ".format(joy_v, joy_w)
        else:
            self.joy_v = self.cmd.linear
            self.joy_w = self.cmd.rotation
            #print "Auto mode: {:.2f} {:.2f}".format(joy_v, joy_w)
        # Apply joystick deadband and calculate vehicle speed (mm/s) and rate of chage of orientation(rad/s)
        if abs(self.joy_v) < self.joyDeadband:
            self.joy_v = 0.0
        else :
            self.joy_v = (1-self.exp) * self.joy_v + (self.exp) * self.joy_v * self.joy_v * self.joy_v
        if abs(self.joy_w) < self.joyDeadband:
            self.joy_w = 0.0
        else :
            self.joy_w = (1-self.exp) * self.joy_w + (self.exp) * self.joy_w * self.joy_w * self.joy_w

        # Apply max Vehicle speed
        Vl = self.joy_v * self.Vl_max_mm_s
        Vr = self.joy_w * self.Vl_max_mm_s
        self.sendCVWcontrol(Vl, Vr)
        
    
    def sendCVWcontrol(self, Vmm_s, Vrad_s):
        if Vmm_s > self.Vl_max_mm_s :
            Vmm_s = self.Vl_max_mm_s
        elif Vmm_s < self.Vl_max_mm_s :
            Vmm_s = -self.Vl_max_mm_s
        # Make a serial message to be sent to motor driver unit
        cmd = '$CVW,{:.0f},{:.0f}'.format(Vmm_s, Vrad_s)
        #print "$CVW: {:.0f} {:.0f} ".format(Vl, Vr)
        if self.ser.isOpen():
            self.ser.write(cmd+"\r"+"\n")
    
    def reset_odometry(self):
        self.last_encoders = {'l': 0, 'r': 0}
        self.pose = PoseWithCovariance()
        self.pose.pose.orientation.w = 1
        
    def setREGI(self, param1, param2):
        msg = "$SREGI,"+str(param1)+','+param2
        self.ser.write(msg+"\r"+"\n")
        print(msg)
        
    def setSPERI(self, param):
        msg = "$SPERI,"+str(param)
        self.ser.write(msg+"\r"+"\n")
        print(msg)
    
    def setPEEN(self, param):
        msg = "$SPEEN,"+str(param)
        self.ser.write(msg+"\r"+"\n")
        print(msg)
        
    def resetODO(self):
        self.ser.write("$SODO\r\n")
        
if __name__ == '__main__':
    try:
        Robot()
    except rospy.ROSInterruptException:
        pass
    

