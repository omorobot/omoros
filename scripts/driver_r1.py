#!/usr/bin/env python
import rospy
import serial

from drive_r1.msg import R1MotorStatus, R1MotorStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from copy import copy, deepcopy

motorL = R1MotorStatus()
motorR = R1MotorStatus()

ser = serial.Serial('/dev/ttyS0', 115200)
rpm_max = 500
joyAxes = []
joyButtons = []
joyDeadband = 0.15
exp = 0.3
isAutoMode = False

from sensor_msgs.msg import Joy
#from my_classifier.msg import Num

#cmd = Num()
def talker():
    #pub_enc_l = rospy.Publisher('motor/encoder/left', Float64, queue_size=10)
    #pub_enc_r = rospy.Publisher('motor/encoder/right', Float64, queue_size=10)
    #pub = rospy.Publisher('motor/status', R1MotorStatusLR, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(rospy.get_param('~hz', 30))
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
        
        
def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.axes[0])
    #print('{:05.2f} {:05.2f}'.format(data.axes[0], data.axes[1]))
    global joyAxes
    global isAutoMode
    joyAxes = deepcopy(data.axes)
    global joyButtons
    # Read the most recent button state
    newJoyButtons = [0,0,0,0,0,0,0,0]
    newJoyButtons = deepcopy(data.buttons)
    # Check if button 1(B) is newly set
    if (newJoyButtons[1]==1) and (newJoyButtons[1]!=joyButtons[1]):
        if isAutoMode!= True:
            isAutoMode = True
        else:
            isAutoMode = False
    # Update button state
    joyButtons = deepcopy(newJoyButtons)

#def callback2(data):
    #rospy.loginfo(rospy.get_caller_id() + "Callback 2 heard %s", data.myid)
    #global cmd
    #cmd = deepcopy(data)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("joy", Joy, callback)
    #rospy.Subscriber("my_classifier", Num, callback2)
    rospy.Timer(rospy.Duration(0.1), mytimer)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    
def mytimer(event):
    global joyAxes
    global joyButtons
    global isAutoMode
    if isAutoMode!= True:
        #joy_v = joyAxes[3]
        #joy_w = joyAxes[2]
        joy_v = joyAxes[1]
        joy_w = joyAxes[0]
        print "Joy mode: {:.2f} {:.2f} ".format(joy_v, joy_w)
    else:
        joy_v = cmd.linear
        joy_w = cmd.rotation
        print "Auto mode: {:.2f} {:.2f}".format(joy_v, joy_w)
    # Apply joystick deadband and calculate left and right wheel speed %
    if abs(joy_v) < joyDeadband:
        joy_v = 0.0
    else :
        joy_v = (1-exp) * joy_v + (exp) * joy_v * joy_v * joy_v
    if abs(joy_w) < joyDeadband:
        joy_w = 0.0
    else :
        joy_w = (1-exp) * joy_w + (exp) * joy_w * joy_w * joy_w

    # Apply max wheel RPM to the left and right wheel
    #Vl = (joy_v + joy_w/2) * rpm_max
    Vl = joy_v * rpm_max
    Vr = joy_w * rpm_max
    
    # Make a serial message to be sent to motor driver unit
    str = '$CVW,{:.0f},{:.0f}'.format(Vl,Vr)
    #print "$CVW: {:.0f} {:.0f} ".format(Vl, Vr)

    if ser.isOpen():
        ser.write(str+"\r"+"\n")

if __name__ == '__main__':
    print(ser.name)         # check which port was really used
    joyAxes = [0,0,0,0,0,0,0,0]
    joyButtons = [0,0,0,0,0,0,0,0]
    if ser.isOpen():
        print("Serial Open")
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

