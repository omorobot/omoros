#!/usr/bin/env python
import sys

import rospy
import serial
import io
import numpy as np

from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from omoros.msg import R1MotorStatusLR, R1MotorStatus
from omoros.msg import R1Command
from tf.transformations import quaternion_about_axis
from tf.broadcaster import TransformBroadcaster

from copy import copy, deepcopy
from sensor_msgs.msg import Joy

class Robot:
    #ser = serial.Serial('/dev/ttyUSB0', 115200)
    ser = serial.Serial('/dev/ttyS0', 115200) #For raspberryPi
    ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                              newline = '\r',
                              line_buffering = True)
    WIDTH = 590.0       #Default Vehicle width in mm
    WHEEL_R = 105.0     #Wheel radius
    WHEEL_MAXV = 1200.0 #Maximum wheel speed in mm/s
    JOY_MAXVL = 500     #Maximum speed in mm/s
    JOY_MAXVW = 314     #Maximum rotational speed in mrad/s
    joyAxes = []
    joyButtons = []
    joyDeadband = 0.15
    exp = 0.3           #Joystick expo setting
    isAutoMode = False
    #initialize data
    cmd_alive = False   #Set to True if subscrived command message has been received
    cmd_mode = 0        #Command mode (0:vel, rot) <--> (1:speedL, speedR)
    cmd_vel = 0.0       #Commanded velocity
    cmd_rot = 0.0       #Commanded rotational speed
    cmd_speedL = 0.0    #Commanded Left wheel speed mm/s
    cmd_speedR = 0.0    #Commanded Right wheel speed mm/s
    cmd_cnt = 0         #Incremental Counter for checking command message
    enc_L = 0           #Left wheel encoder count from QENCOD message
    enc_R = 0           #Right wheel encoder count from QENCOD message
    odo_L = 0           #Left Wheel odometry returned from QODO message
    odo_R = 0           #Right Wheel odometry returned from QODO message
    RPM_L = 0           #Left Wheel RPM returned from QRPM message
    RPM_R = 0           #Right Wheel RPM returned from QRPM message
    speedL = 0.0        #Left Wheel speed returned from QDIFF message
    speedR = 0.0        #Reft Wheel speed returned from QDIFF message
    vel = 0.0           #Velocity returned from CVW command
    rot = 0.0           #Rotational speed returned from CVR command
    def __init__(self, arg):
        if arg == "r1":
            print "**********"
            print "Driving R1"
            print "**********"
        if arg == "mini":
            print "***************"
            print "Driving R1-mini"
            print "***************"
            self.WIDTH = 170.0      #Apply vehicle width for mini version
            self.WHEEL_R = 33.6     #Apply wheel radius for mini version
        else :
            print "Only support r1 and mini. exit..."
            exit()
        
        print(self.ser.name)         # check which port was really used
        self.joyAxes = [0,0,0,0,0,0,0,0]
        self.joyButtons = [0,0,0,0,0,0,0,0]
        # Configure data output
        if self.ser.isOpen():
            print("Serial Open")
            self.resetODO()
            self.setREGI(0,'QENCOD')
            self.setREGI(1,'QODO')
            self.setREGI(2,'QDIFFV')
            self.setSPERI(20)
            self.setPEEN(1)
            
        self.reset_odometry()   
        rospy.init_node('omoros', anonymous=True)
        
        self.br = TransformBroadcaster()
        # Subscriber
        rospy.Subscriber("joy", Joy, self.callbackJoy)
        rospy.Subscriber("R1Command", R1Command, self.callbackCommand)
        # publisher
        self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
        self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
        self.pub_odometry = rospy.Publisher("odom", Odometry, queue_size=10)
        self.pub_motor_status = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
        self.pub_joints = rospy.Publisher("joint_state", JointState, queue_size=10)
        
        rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
        rospy.Timer(rospy.Duration(0.1), self.joytimer)
        rospy.Timer(rospy.Duration(0.01), self.serReader)
        
        while not rospy.is_shutdown():
            if self.cmd_alive == True :
                self.cmd_cnt+=1
                if self.cmd_cnt > 1000 : #Wait for about 3 seconds 
                    self.cmd_alive = False
                    self.isAutoMode = False
            rate.sleep()
            
        self.ser.close()

    def serReader(self, event):
        reader = self.ser_io.readline()
        if reader:
            packet = reader.split(",")
            try:
                header = packet[0].split("#")[1]
                if header.startswith('CVW'):
                    self.vel = int(packet[1])
                    self.rot = int(packet[2])

                elif header.startswith('QENCOD'):
                    self.enc_L = int(packet[1])
                    self.enc_R = int(packet[2])
                    self.pub_enc_l.publish(Float64(data=self.enc_L))
                    self.pub_enc_r.publish(Float64(data=self.enc_R))
                    #print('{:04d},{:04d}'.format(self.enc_L, self.enc_R))
                elif header.startswith('QODO'):
                    self.odo_L = int(packet[1])
                    self.odo_R = int(packet[2])
                    # publish current pose
                    (odom, transform)= self.odometry(self.odo_L/1000.0, self.odo_R/1000.0)
                    self.pub_odometry.publish(odom)
                    self.br.sendTransformMessage(transform)
                elif header.startswith('QRPM'):
                    self.RPM_L = int(packet[1])
                    self.RPM_R = int(packet[2])
                    #print('{:04d},{:04d}'.format(self.RPM_L, self.RPM_R))
                elif header.startswith('QDIFFV'):
                    self.speedL = int(packet[1])
                    self.speedR = int(packet[2])
                    # publish current pose
                    #(odom, transform)= self.odometry(self.speedL/1000.0, self.speedR/1000.0)
                    #self.pub_odometry.publish(odom)
                    #self.br.sendTransformMessage(transform)
                    #print('{:04d},{:04d}'.format(self.wheelL_mm_s, self.wheelR_mm_s))
            except:
                print('Wrong packet')
                pass
                
            status_left = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                                        encoder = self.enc_L, RPM = self.RPM_L, ODO = self.odo_L, speed = self.speedL)
            status_right = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                                        encoder = self.enc_R, RPM = self.RPM_R, ODO = self.odo_R, speed = self.speedR)
            self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()), left=status_left, right=status_right))        

            
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
    
    def callbackCommand(self, data):
        if self.cmd_alive != True:
            self.cmd_alive = True
        self.cmd_cnt = 0        #Reset counter
        self.mode = data.mode
        self.vel = data.velocity
        self.rot = data.rotation
        if mode == 0:   #Control by velocity and twist
            (self.cmd_speedL, self.cmd_speedR) = self.getWheelSpeed(vel, rot)
        else :
            self.cmd_speedL = lSpeed
            self.cmd_speedR = rSpeed
        if self.isAutoMode == True:
            print "AutoMode VL, VR: {:.2f} {:.2f}".format(speedL, speedR)
            self.sendCDIFFVcontrol(speedL, speedR)

    def reset_odometry(self):
        self.last_speedL = 0.0
        self.last_speedR = 0.0
        self.pose = PoseWithCovariance()
        self.pose.pose.orientation.w = 1
        
    def odometry(self, left, right):
        currentTime = rospy.Time.now();
        lSpeed = left
        rSpeed = right
        # Compute current linear and angular speed from wheel speed
        twist = TwistWithCovariance()
        twist.twist.linear.x = (rSpeed + lSpeed) / 2.0
        twist.twist.angular.z = (rSpeed - lSpeed) / self.WIDTH
        # Compute position and orientation from travelled distance per wheel
        dl = (lSpeed - self.last_speedL)
        dr = (rSpeed - self.last_speedR)
        # set previous encoder state
        self.last_speedL = lSpeed
        self.last_speedR = rSpeed
        angle = (dr-dl) / self.WIDTH
        linear = 0.5*(dl+dr)
        if dr!=dl:
            radius = self.WIDTH/2.0 * (dl+dr) / (dr-dl)
        else:
            radius = 0

        # old state
        old_angle = 2*np.arccos(self.pose.pose.orientation.w)
        old_pos = np.array([self.pose.pose.position.x, self.pose.pose.position.y])

        # update state
        new_angle = (old_angle+angle) % (2*np.pi)
        new_q = quaternion_about_axis(new_angle, (0, 0, 1))
        new_angle2 = 2 * np.arccos(self.pose.pose.orientation.w)
        #print("new_angle2", new_angle2)
        new_pos = np.zeros((2,))

        if abs(angle) < 1e-6:
            direction = old_angle + angle * 0.5
            dx = linear * np.cos(direction)
            dy = linear * np.sin(direction)
        else:
            dx = + radius * (np.sin(new_angle) - np.sin(old_angle))
            dy = - radius * (np.cos(new_angle) - np.cos(old_angle))

        new_pos[0] = old_pos[0] + dx
        new_pos[1] = old_pos[1] + dy

        self.pose.pose.orientation.x = new_q[0]
        self.pose.pose.orientation.y = new_q[1]
        self.pose.pose.orientation.z = new_q[2]
        self.pose.pose.orientation.w = new_q[3]
        self.pose.pose.position.x = new_pos[0]
        self.pose.pose.position.y = new_pos[1]

        odom = Odometry(header=Header(stamp=rospy.Time.now(), frame_id="odom"), child_frame_id="base_link",
                        pose=self.pose, twist=twist)

        transform = TransformStamped(header=Header(stamp=rospy.Time.now(), frame_id="world"), child_frame_id="omoros")
        transform.transform.translation.x = self.pose.pose.position.x
        transform.transform.translation.y = self.pose.pose.position.y
        transform.transform.translation.z = self.pose.pose.position.z
        transform.transform.rotation = self.pose.pose.orientation

        return odom, transform

    def joytimer(self, event):
        if self.isAutoMode!= True:
            self.joy_v = self.joyAxes[1]
            self.joy_w = self.joyAxes[0]
            #print "Joy mode: {:.2f} {:.2f} ".format(self.joy_v, self.joy_w)
        else:
            return
        # Apply joystick deadband and calculate vehicle speed (mm/s) and rate of chage of orientation(rad/s)
        joyV = 0.0
        joyR = 0.0
        if abs(self.joy_v) < self.joyDeadband:
            joyV = 0.0
        else :
            joyV = (1-self.exp) * self.joy_v + (self.exp) * self.joy_v * self.joy_v * self.joy_v
        if abs(self.joy_w) < self.joyDeadband:
            joyR = 0.0
        else :
            joyR = (1-self.exp) * self.joy_w + (self.exp) * self.joy_w * self.joy_w * self.joy_w
        # Apply max Vehicle speed
        (speedL, speedR) = self.getWheelSpeed(joyV * self.JOY_MAXVL, joyR * self.JOY_MAXVW)
        print "Joystick VL, VR: {:.2f} {:.2f}".format(speedL, speedR)
        self.sendCDIFFVcontrol(speedL, speedR)
    
    def getWheelSpeed(self, V, W):
        #Calculate left and right wheel speed from Velocity, Rotation speed        
        #Kinematics reference from http://enesbot.me/kinematic-model-of-a-differential-drive-robot.html
        speedL = 2.0*V - (self.WIDTH * W / self.WHEEL_R + 2.0 * V)/2.0
        speedR = (self.WIDTH * W / self.WHEEL_R + 2.0*V) / 2.0
        return speedL, speedR

    def sendCVWcontrol(self, Vmm_s, Vrad_s):
        #Set Vehicle velocity and rotational speed
        if Vmm_s > self.JOY_MAXVL :
            Vmm_s = self.JOY_MAXVL
        elif Vmm_s < -self.JOY_MAXVL :
            Vmm_s = -self.JOY_MAXVL
        # Make a serial message to be sent to motor driver unit
        cmd = '$CVW,{:.0f},{:.0f}'.format(Vmm_s, Vrad_s)
        #print "$CVW: {:.0f} {:.0f} ".format(Vmm_s, Vrad_s)
        if self.ser.isOpen():
            self.ser.write(cmd+"\r"+"\n")

    def sendCDIFFVcontrol(self, VLmm_s, VRmm_s):
        #Set differential wheel speed for Left and Right
        if VLmm_s > self.WHEEL_MAXV :
            VLmm_s = self.WHEEL_MAXV
        elif VLmm_s < -self.WHEEL_MAXV :
            VLmm_s = -self.WHEEL_MAXV
        if VRmm_s > self.WHEEL_MAXV :
            VRmm_s = self.WHEEL_MAXV
        elif VRmm_s < -self.WHEEL_MAXV :
            VRmm_s = -self.WHEEL_MAXV
        # Make a serial message to be sent to motor driver unit
        cmd = '$CDIFFV,{:.0f},{:.0f}'.format(VLmm_s, VRmm_s)
        #print "$CVW: {:.0f} {:.0f} ".format(Vmm_s, Vrad_s)
        if self.ser.isOpen():
            self.ser.write(cmd+"\r"+"\n")
                    

    def setREGI(self, param1, param2):
        msg = "$SREGI,"+str(param1)+','+param2
        self.ser.write(msg+"\r"+"\n")
        #print(msg)
        
    def setSPERI(self, param):
        msg = "$SPERI,"+str(param)
        self.ser.write(msg+"\r"+"\n")
        #print(msg)
    
    def setPEEN(self, param):
        msg = "$SPEEN,"+str(param)
        self.ser.write(msg+"\r"+"\n")
        #print(msg)
        
    def resetODO(self):
        self.ser.write("$SODO\r\n")
        
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print "Must enter either mini or r1"
        exit()
    try:
        Robot(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
    

