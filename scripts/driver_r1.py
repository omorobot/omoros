#!/usr/bin/env python

"""driver_r1.py: ROS driver for Omorobot R1 and R1-mini"""
# For more information, please visit our website www.omorobot.com
# Want to discuss with developers using our robots? Please visit our forum website at http://omorobot1.synology.me
# Also note that this software is for experimental and subject to change
# without any notifications.
__license__ = "MIT"
__version__ = "0.1.3"
__status__ = "Experimental"
'''
## License
The MIT License (MIT)
R1 and R1 mini driver for ROS: an open source platform for driving a robot with ROS.
Copyright (C) 2019  OMOROBOT Inc
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
'''
import sys
import rospy
import serial
import io
import numpy as np
import math

from time import sleep
from std_msgs.msg import UInt8, Int8, Int16, Float64, Float32
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from omoros.msg import R1MotorStatusLR, R1MotorStatus
from omoros.msg import R1Command

from copy import copy, deepcopy
from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist, TwistWithCovariance, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class MyPose(object):
   x = 0
   y = 0
   theta = 0
   timestamp = 0

class ArrowCon:
   setFwd = 0           # 1:Fwd, -1: Rev
   setRot = 0           # 1:CCW(Turn Left), -1: CW(Turn Right)
   startOdo_L = 0       # Odometry when started
   startOdo_R = 0
   targetOdo_L = 0      # Odometry target
   targetOdo_R = 0
   isFinished = True    # True: If arrow motion is completed
   cnt = 0
   
class Encoder(object):
   Dir = 1.0
   PPR = 0
   GearRatio = 0
   Step = 0
   
class VehicleConfig(object):
   BodyCircumference = 0   # circumference length of robot for spin in place
   WheelCircumference = 0
   WIDTH = 0.0             # Default Vehicle width in mm
   WHEEL_R = 0.0           # Wheel radius
   WHEEL_MAXV = 0.0        #    
   JOY_MAXVspeed = 0       # Maximum speed in mm/s
   JOY_MAXVomega = 0       # Maximum rotational speed in mrad/s
   ArrowFwdStep = 100.0    # Forward motion step when arrow key pressed (mm)
   ArrowRotRate = 1/10.0   # Rotational rate per full turn
   encoder = Encoder()
   
class Command:
   isAlive = False   # Set to True if subscrived command message has been received
   mode = 0          # Command mode (0:vel, rot) <--> (1:speedL, speedR)
   speed = 0.0       # Speed mm/s
   deg_sec = 0.0     # Rotational speed deg/s
   speedL = 0.0      # Left Wheel speed mm/s
   speedR = 0.0      # Right wheel speed mm/s

class Robot:
   ser = serial.Serial('/dev/ttyUSB0', 115200)
   #ser = serial.Serial('/dev/ttyS0', 115200) #For raspberryPi
   ser_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser, 1),
                           newline = '\r',
                           line_buffering = True)
   config = VehicleConfig()
   pose = MyPose()
   joyAxes = []
   joyButtons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    # Buttons 15
   joyDeadband = 0.15
   exp = 0.3            # Joystick expo setting
   isAutoMode = False
   isArrowMode = False  # Whether to control robo with arrow key or not
   arrowCon = ArrowCon
   
   #initialize data
   cmd = Command
   enc_L = 0.0          # Left wheel encoder count from QENCOD message
   enc_R = 0.0          # Right wheel encoder count from QENCOD message
   enc_L_prev = 0.0
   enc_R_prev = 0.0
   enc_offset_L = 0.0
   enc_offset_R = 0.0
   enc_cnt = 0
   odo_L = 0.0          # Left Wheel odometry returned from QODO message
   odo_R = 0.0          # Right Wheel odometry returned from QODO message
   RPM_L = 0.0          # Left Wheel RPM returned from QRPM message
   RPM_R = 0.0          # Right Wheel RPM returned from QRPM message
   speedL = 0.0         # Left Wheel speed returned from QDIFF message
   speedR = 0.0         # Reft Wheel speed returned from QDIFF message
   vel = 0.0            # Velocity returned from CVW command
   rot = 0.0            # Rotational speed returned from CVR command
   def __init__(self, arg):
      ## Set vehicle specific configurations
      if arg == "r1":
         print "**********"
         print "Driving R1"
         print "**********"
         self.config.WIDTH = 0.591      # Apply vehicle width for R1 version
         self.config.WHEEL_R = 0.11      # Apply wheel radius for R1 version
         self.config.WHEEL_MAXV = 1200.0
         self.config.JOY_MAXVspeed = 300
         self.config.JOY_MAXVomega = 50
         self.config.ArrowFwdStep = 250   # Steps move forward based on Odometry
         self.config.ArrowRotRate = 1/8
         self.config.encoder.Dir = 1.0
         self.config.encoder.PPR = 1000
         self.config.encoder.GearRatio = 15
         
      elif arg == "mini":
         print "***************"
         print "Driving R1-mini"
         print "***************"
         self.config.WIDTH = 0.170      # Apply vehicle width for mini version
         self.config.WHEEL_R = 0.0336     # Apply wheel radius for mini version
         self.config.WHEEL_MAXV = 500.0
         self.config.JOY_MAXVspeed = 500.0
         self.config.JOY_MAXVomega = 100
         self.config.ArrowFwdStep = 100
         self.config.ArrowRotRate = 1/10
         self.config.encoder.Dir = -1.0
         self.config.encoder.PPR = 234
         self.config.encoder.GearRatio = 21         
      else :
         print "Only support r1 and mini. exit..."
         exit()
      print('Wheel Width:{:.2f}m, Radius:{:.2f}m'.format(self.config.WIDTH, self.config.WHEEL_R))
      self.config.BodyCircumference = self.config.WIDTH * math.pi
      print('Platform Rotation arc length: {:04f}m'.format(self.config.BodyCircumference))
      self.config.WheelCircumference = self.config.WHEEL_R * 2 * math.pi
      print('Wheel circumference: {:04f}m'.format(self.config.WheelCircumference))
      self.config.encoder.Step = self.config.WheelCircumference / (self.config.encoder.PPR * self.config.encoder.GearRatio * 4)
      print('Platform encoder step: {:04f}m/pulse'.format(self.config.encoder.Step))
      
      print(self.ser.name)         # Print which port was really used
      self.joyAxes = [0,0,0,0,0,0,0,0]
      self.joyButtons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      # Configure data output
      if self.ser.isOpen():
         print("Serial Open")
         self.resetODO()
         sleep(0.05)
         self.reset_odometry()
         self.setREGI(0,'QENCOD')
         sleep(0.05)
         self.setREGI(1,'QODO')
         sleep(0.05)
         self.setREGI(2,'QDIFFV')
         sleep(0.05)
         self.setREGI(3,'QVW')
         sleep(0.05)
         self.setREGI(4,'QRPM')
         sleep(0.05)
         self.setSPERI(20)
         sleep(0.05)
         self.setPEEN(1)
         sleep(0.05)
         
      self.reset_odometry()   
      rospy.init_node('omoros', anonymous=True)
      
      self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()
      
      # Subscriber
      rospy.Subscriber("joy", Joy, self.callbackJoy)
      rospy.Subscriber("R1Command", R1Command, self.callbackCommand)
      # publisher
      self.pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
      self.pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
      self.pub_motor_status = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)

      rate = rospy.Rate(rospy.get_param('~hz', 30)) # 30hz
      rospy.Timer(rospy.Duration(0.05), self.joytimer)
      rospy.Timer(rospy.Duration(0.01), self.serReader)
      self.pose.timestamp = rospy.Time.now()

      while not rospy.is_shutdown():
         if self.cmd.isAlive == True:
             self.cmd.cnt += 1
             if self.cmd.cnt > 1000:         #Wait for about 3 seconds 
                 self.cmd.isAlive = False
                 self.isAutoMode = False
         rate.sleep()
         
      self.ser.close()

   def serReader(self, event):
      reader = self.ser_io.readline()
      if reader:
         packet = reader.split(",")
         try:
            header = packet[0].split("#")[1]
            if header.startswith('QVW'):
               self.vel = int(packet[1])
               self.rot = int(packet[2])
            elif header.startswith('QENCOD'):
               enc_L = int(packet[1])
               enc_R = int(packet[2])
               if self.enc_cnt == 0:
                  self.enc_offset_L = enc_L
                  self.enc_offset_R = enc_R
               self.enc_cnt+=1
               self.enc_L = enc_L*self.config.encoder.Dir - self.enc_offset_L
               self.enc_R = enc_R*self.config.encoder.Dir - self.enc_offset_R
               self.pub_enc_l.publish(Float64(data=self.enc_L))
               self.pub_enc_r.publish(Float64(data=self.enc_R))
               self.pose = self.updatePose(self.pose, self.enc_L, self.enc_R)
               #print('Encoder:L{:.2f}, R:{:.2f}'.format(self.enc_L, self.enc_R))
            elif header.startswith('QODO'):
               self.odo_L = float(packet[1])*self.config.encoder.Dir
               self.odo_R = float(packet[2])*self.config.encoder.Dir
               #print('Odo:{:.2f}mm,{:.2f}mm'.format(self.odo_L, self.odo_R))
            elif header.startswith('QRPM'):
               self.RPM_L = int(packet[1])
               self.RPM_R = int(packet[2])
               #print('RPM:{:.2f}mm,{:.2f}mm'.format(self.RPM_L, self.RPM_R))
            elif header.startswith('QDIFFV'):
               self.speedL = int(packet[1])
               self.speedR = int(packet[2])
         except:
            pass
         status_left = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                           encoder = self.enc_L, RPM = self.RPM_L, ODO = self.odo_L, speed = self.speedL)
         status_right = R1MotorStatus(low_voltage = 0, overloaded = 0, power = 0,
                           encoder = self.enc_R, RPM = self.RPM_R, ODO = self.odo_R, speed = self.speedR)
         self.pub_motor_status.publish(R1MotorStatusLR(header=Header(stamp=rospy.Time.now()),
                           Vspeed = self.vel, Vomega = self.rot,
                           left=status_left, right=status_right))        

            
   def callbackJoy(self, data):
      self.joyAxes = deepcopy(data.axes)
      # Read the most recent button state
      newJoyButtons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
      newJoyButtons = deepcopy(data.buttons)
      # Check if button 1(B) is newly set
      if (newJoyButtons[1]==1) and (newJoyButtons[1]!=self.joyButtons[1]):
         if self.isAutoMode!= True:
             self.isAutoMode = True
         else:
             self.isAutoMode = False
             
      if (newJoyButtons[9]==1) and (newJoyButtons[9]!=self.joyButtons[9]):
         if self.isArrowMode!= True:
             self.isArrowMode = True
             self.arrowCon.isFinished = True
             print "Joystick Arrow Mode"
         else:
             self.isArrowMode = False
             print "Joystick Axis Mode"
      
      if self.isArrowMode == True:
         #if (newJoyButtons[13]==1) or (newJoyButtons[14]==1):
         if (self.joyAxes[7]==1.0) or (self.joyAxes[7]==-1.0):
            if self.arrowCon.isFinished ==True:
               self.arrowCon.isFinished = False
               self.arrowCon.startOdo_L = self.odo_L
               self.arrowCon.startOdo_R = self.odo_R
               #if newJoyButtons[13]==1:   # FWD arrow
               if self.joyAxes[7]==1.0:
                  self.arrowCon.setFwd = 1
                  self.arrowCon.targetOdo_L = self.odo_L + self.config.ArrowFwdStep #target 1 step ahead
                  self.arrowCon.targetOdo_R = self.odo_R + self.config.ArrowFwdStep #target 1 step ahead
                  print "Arrow Fwd"
               else:                      # REV arrow
                  self.arrowCon.setFwd = -1 
                  self.arrowCon.targetOdo_L = self.odo_L - self.config.ArrowFwdStep #target 1 step rear
                  self.arrowCon.targetOdo_R = self.odo_R - self.config.ArrowFwdStep #target 1 step rear
                  print "Arrow Rev"
               print "Arrow: {:.2f} {:.2f} ".format(self.arrowCon.startOdo_L, self.arrowCon.targetOdo_L)
         
         #elif (newJoyButtons[11]==1) or (newJoyButtons[12]==1):
         elif (self.joyAxes[6]==1.0) or (self.joyAxes[6]==-1.0):
            if self.arrowCon.isFinished ==True:
               turnRate = 10.5
               self.arrowCon.isFinished = False
               self.arrowCon.startOdo_L = self.odo_L
               self.arrowCon.startOdo_R = self.odo_R
               #if newJoyButtons[11]==1:   # Left arrow
               if self.joyAxes[6]==1.0:
                  self.arrowCon.setRot = 1
                  self.arrowCon.targetOdo_L = self.odo_L + self.config.BodyCircumference*self.config.ArrowRotRate*self.config.encoder.Dir
                  self.arrowCon.targetOdo_R = self.odo_R - self.config.BodyCircumference*self.config.ArrowRotRate*self.config.encoder.Dir
                  print "Arrow Left"
               else:                     # Right arrow
                  self.arrowCon.setRot = -1
                  self.arrowCon.targetOdo_L = self.odo_L - self.config.BodyCircumference*self.config.ArrowRotRate*self.config.encoder.Dir
                  self.arrowCon.targetOdo_R = self.odo_R + self.config.BodyCircumference*self.config.ArrowRotRate*self.config.encoder.Dir
                  print "Arrow Right"
      # Update button state
      self.joyButtons = deepcopy(newJoyButtons)

   def callbackCommand(self, data):
      if self.cmd.isAlive != True:
         self.cmd.isAlive = True
      self.cmd.cnt = 0        #Reset counter
      self.mode = data.mode
      self.vel = data.velocity
      self.rot = data.rotation
      if mode == 0:   #Control by velocity and twist
         (self.cmd.speedL, self.cmd.speedR) = self.getWheenSpeed(vel, rot) 
      else :
         self.cmd.speedL = lSpeed
         self.cmd.speedR = rSpeed
      if self.isAutoMode == True:
         self.sendCDIFFVcontrol(speedL, speedR)

   def reset_odometry(self):
      self.last_speedL = 0.0
      self.last_speedR = 0.0

   def joytimer(self, event):
      if self.isAutoMode!= True:
         self.joy_v = self.joyAxes[1]
         self.joy_w = self.joyAxes[0]
         #print "Joy mode: {:.2f} {:.2f} ".format(self.joy_v, self.joy_w)
      else:
         return
      if not self.isArrowMode:
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
         (speedL, speedR) = self.getWheelSpeed(self.config, joyV * self.config.JOY_MAXVspeed, joyR * self.config.JOY_MAXVomega)
         #print "Joystick VL, VR: {:.2f} {:.2f}".format(speedL, speedR)
         self.sendCDIFFVcontrol(speedL, speedR)
      else:
         if self.arrowCon.isFinished == False:
            if self.arrowCon.setFwd == 1:  # For forward motion
               if (self.odo_L < self.arrowCon.targetOdo_L) or (self.odo_R < self.arrowCon.targetOdo_R ):
                  print "Fwd: {:.2f} {:.2f} ".format(self.odo_L, self.odo_R)
                  self.sendCDIFFVcontrol(100, 100)
               else:
                  self.sendCDIFFVcontrol(0, 0)
                  self.arrowCon.isFinished = True
                  self.arrowCon.setFwd = 0
                  print "Finished!"
            elif self.arrowCon.setFwd == -1:
               if (self.odo_L > self.arrowCon.targetOdo_L ) or (self.odo_R > self.arrowCon.targetOdo_R ):
                  print "Rev: {:.2f} {:.2f} ".format(self.odo_L, self.odo_R)
                  self.sendCDIFFVcontrol(-100, -100)
               else:
                  self.sendCDIFFVcontrol(0, 0)
                  self.arrowCon.isFinished = True
                  self.arrowCon.setFwd = 0
                  print "Finished!"
            elif self.arrowCon.setRot == 1:
               if (self.odo_L > self.arrowCon.targetOdo_L) or (self.odo_R < self.arrowCon.targetOdo_R):
                  print "Rev: {:.2f} {:.2f} ".format(self.odo_L, self.odo_R)
                  self.sendCDIFFVcontrol(-100, 100)
               else:
                  self.sendCDIFFVcontrol(0, 0)
                  self.arrowCon.isFinished = True
                  self.arrowCon.setRot = 0
                  print "Finished!"
            elif self.arrowCon.setRot == -1:
               if (self.odo_L < self.arrowCon.targetOdo_L) or (self.odo_R > self.arrowCon.targetOdo_R):
                  print "Rev: {:.2f} {:.2f} ".format(self.odo_L, self.odo_R)
                  self.sendCDIFFVcontrol(100, -100)
               else:
                  self.sendCDIFFVcontrol(0, 0)
                  self.arrowCon.isFinished = True
                  self.arrowCon.setRot = 0
                  print "Finished!"
                  
   def updatePose(self, pose, encoderL, encoderR):
      now = rospy.Time.now()
      dL = encoderL - self.enc_L_prev
      dR = encoderR - self.enc_R_prev
      self.enc_L_prev = encoderL
      self.enc_R_prev = encoderR
      dT = (now - pose.timestamp)/1000000.0
      pose.timestamp = now
      x = pose.x
      y = pose.y
      theta = pose.theta
      R = 0.0
      if (dR-dL)==0:
         R = 0.0
      else:
         R = self.config.WIDTH/2.0*(dL+dR)/(dR-dL)
      Wdt = (dR - dL) * self.config.encoder.Step / self.config.WIDTH

      ICCx = x - R * np.sin(theta)
      ICCy = y + R * np.cos(theta)
      pose.x = np.cos(Wdt)*(x - ICCx) - np.sin(Wdt)*(y - ICCy) + ICCx
      pose.y = np.sin(Wdt)*(x - ICCx) + np.cos(Wdt)*(y - ICCy) + ICCy
      pose.theta = theta + Wdt
      
      twist = TwistWithCovariance()
      
      twist.twist.linear.x = self.vel/1000.0
      twist.twist.linear.y = 0.0
      twist.twist.angular.z = self.rot/1000.0
      
      Vx = twist.twist.linear.x
      Vy = twist.twist.linear.y
      Vth = twist.twist.angular.z
      odom_quat = quaternion_from_euler(0,0,pose.theta)
      self.odom_broadcaster.sendTransform((pose.x,pose.y,0.),odom_quat,now,'base_link','odom')
      
      odom = Odometry()
      odom.header.stamp = now
      odom.header.frame_id = 'odom'
      odom.pose.pose = Pose(Point(pose.x,pose.y,0.),Quaternion(*odom_quat))
      
      odom.child_frame_id = 'base_link'
      odom.twist.twist = Twist(Vector3(Vx,Vy,0),Vector3(0,0,Vth))
      #print "x,y: {:.2f} {:.2f} {:.2f}".format(pose.x, pose.y, pose.theta*180/math.pi)      
      self.odom_pub.publish(odom)
      return pose

   def getWheelSpeed(self, config, V, W):
      #Calculate left and right wheel speed from Velocity, Rotation speed        
      #Kinematics reference from http://enesbot.me/kinematic-model-of-a-differential-drive-robot.html
      speedL = V - config.WIDTH * W / config.WHEEL_R /2.0
      speedR = V + config.WIDTH * W / config.WHEEL_R /2.0
      return speedL, speedR

   def sendCVWcontrol(self, config, Vmm_s, Vrad_s):
      #Set Vehicle velocity and rotational speed
      if Vmm_s > config.JOY_MAXVspeed :
         Vmm_s = config.JOY_MAXVspeed
      elif Vmm_s < -config.JOY_MAXVspeed :
         Vmm_s = -config.JOY_MAXVspeed
      # Make a serial message to be sent to motor driver unit
      cmd = '$CVW,{:.0f},{:.0f}'.format(Vmm_s, Vrad_s)
      #print "$CVW: {:.0f} {:.0f} ".format(Vmm_s, Vrad_s)
      if self.ser.isOpen():
         self.ser.write(cmd+"\r"+"\n")

   def sendCDIFFVcontrol(self, VLmm_s, VRmm_s):
      #Set differential wheel speed for Left and Right
      if VLmm_s > self.config.WHEEL_MAXV :
         VLmm_s = self.config.WHEEL_MAXV
      elif VLmm_s < -self.config.WHEEL_MAXV :
         VLmm_s = -self.config.WHEEL_MAXV
      if VRmm_s > self.config.WHEEL_MAXV :
         VRmm_s = self.config.WHEEL_MAXV
      elif VRmm_s < -self.config.WHEEL_MAXV :
         VRmm_s = -self.config.WHEEL_MAXV
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
    

