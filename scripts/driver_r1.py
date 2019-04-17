#!/usr/bin/env python
import sys

import rospy
import serial
import io
import numpy as np
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import TransformStamped

from nav_msgs.msg import Odometry
from drive_r1.msg import R1MotorStatusLR, R1MotorStatus
#from tf.transformations import quaternion_about_axis
#from tf.broadcaster import TransformBroadcaster
from copy import copy, deepcopy
from sensor_msgs.msg import Joy

class Robot:
    ser = serial.Serial('/dev/ttyS0', 115200)
    ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                              newline = '\r',
                              line_buffering = True)
    rpm_max = 500
    joyAxes = []
    joyButtons = []
    joyDeadband = 0.15
    exp = 0.3
    rcvCnt = 0
    isAutoMode = False
    
    def __init__(self):
        print(self.ser.name)         # check which port was really used
        self.joyAxes = [0,0,0,0,0,0,0,0]
        self.joyButtons = [0,0,0,0,0,0,0,0]
        if self.ser.isOpen():
            print("Serial Open")
            msg = '$SREGI,0,ENCOD'
            self.ser.write(msg+"\r"+"\n")
            
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
        #rospy.Subscriber("my_classifier", Num, callback2)
        rospy.Timer(rospy.Duration(0.1), self.joytimer)
        rospy.Timer(rospy.Duration(0.1), self.reader)
        
        while not rospy.is_shutdown():
            

            rate.sleep()
        self.ser.close()
    
    def talker():
        #pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        #pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
        #pub = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
        rospy.init_node('omoros', anonymous=True)
        rate = rospy.Rate(rospy.get_param('~hz', 30))
        print("Talker Init")
        print("connected to: " + ser.portstr)
        count=1
        while not rospy.is_shutdown():
            for line in ser.read():
                print(str(count) + str(': ') + chr(line) )
            count = count+1
            #data= ser.read(2) # I have "hi" coming from the arduino as a test run over the serial port
            # publish motor status, including encoder value
            #(flags, power, encoder, speed) = self.g.get_motor_status(self.ML)
            #status_left = MotorStatus(low_voltage=(flags & (1<<0)), overloaded=(flags & (1<<1)),
            #                          power=power, encoder=encoder, speed=speed)
            #self.pub_enc_l.publish(Float64(data=encoder))

            #(flags, power, encoder, speed) = self.g.get_motor_status(self.MR)
            #status_right = MotorStatus(low_voltage=(flags & (1<<0)), overloaded=(flags & (1<<1)),
            #                          power=power, encoder=encoder, speed=speed)
            #self.pub_enc_r.publish(Float64(data=encoder))

            #self.pub_motor_status.publish(MotorStatusLR(header=Header(stamp=rospy.Time.now()), left=status_left, right=status_right))
            
            # publish current pose
            #(odom, transform)= self.odometry(status_left, status_right)
            #self.pub_odometry.publish(odom)
            #self.br.sendTransformMessage(transform)
            rate.sleep()
            
            
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

    def reader(self, event):
        reader = self.ser_io.readline()
        if reader:
            packet = reader.split(",")
            header = packet[0].split("#")[1]
            #print(reader)
            if header.startswith('CVW'):
                encoderL = int(packet[1])
                encoderR = int(packet[2])
                print('{:04d},{:04d}'.format(encoderL, encoderR))
                # publish motor status, including encoder value
                status_left = R1MotorStatus(low_voltage=0, overloaded=0,
                                      power=0, encoder=encoderL, speed=encoderL)
                status_right = R1MotorStatus(low_voltage=0, overloaded=0,
                                      power=0, encoder=encoderR, speed=encoderR)
                self.pub_enc_l.publish(Float64(data=encoderL))
                self.pub_enc_r.publish(Float64(data=encoderR))
                self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()), left=status_left, right=status_right))
        
    def joytimer(self, event):
        #global joyAxes
        #global joyButtons
        #global isAutoMode
        if self.isAutoMode!= True:
            #joy_v = joyAxes[3]
            #joy_w = joyAxes[2]
            self.joy_v = self.joyAxes[1]
            self.joy_w = self.joyAxes[0]
            #print "Joy mode: {:.2f} {:.2f} ".format(joy_v, joy_w)
        else:
            self.joy_v = self.cmd.linear
            self.joy_w = self.cmd.rotation
            #print "Auto mode: {:.2f} {:.2f}".format(joy_v, joy_w)
        # Apply joystick deadband and calculate left and right wheel speed %
        if abs(self.joy_v) < self.joyDeadband:
            self.joy_v = 0.0
        else :
            self.joy_v = (1-self.exp) * self.joy_v + (self.exp) * self.joy_v * self.joy_v * self.joy_v
        if abs(self.joy_w) < self.joyDeadband:
            self.joy_w = 0.0
        else :
            self.joy_w = (1-self.exp) * self.joy_w + (self.exp) * self.joy_w * self.joy_w * self.joy_w

        # Apply max wheel RPM to the left and right wheel
        #Vl = (joy_v + joy_w/2) * rpm_max
        Vl = self.joy_v * self.rpm_max
        Vr = self.joy_w * self.rpm_max
        
        # Make a serial message to be sent to motor driver unit
        str = '$CVW,{:.0f},{:.0f}'.format(Vl,Vr)
        #print "$CVW: {:.0f} {:.0f} ".format(Vl, Vr)

        if self.ser.isOpen():
            self.ser.write(str+"\r"+"\n")
            
    def reset_odometry(self):
        #self.g.offset_motor_encoder(self.ML, self.g.get_motor_encoder(self.ML))
        #self.g.offset_motor_encoder(self.MR, self.g.get_motor_encoder(self.MR))
        self.last_encoders = {'l': 0, 'r': 0}
        self.pose = PoseWithCovariance()
        self.pose.pose.orientation.w = 1

if __name__ == '__main__':
    try:
        Robot()
    except rospy.ROSInterruptException:
        pass
    

