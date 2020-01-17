#!/usr/bin/env python

import serial
import rospy
import numpy as np
import time
from std_msgs.msg import UInt8MultiArray 
from sensor_msgs.msg import Joy 
        


class TeleopDriver:

    def __init__(self):

        rospy.init_node("motor_control_node")
       
        self.sub = rospy.Subscriber("j0/joy_throttle",Joy, self.callback) 
        self.pub = rospy.Publisher("motor", UInt8MultiArray, queue_size=10) 
        rospy.spin()   

    def callback(self, data):
        #print(data)
        axes = data.axes 

        sL = int(axes[5]*255.)
        sR = int(axes[1]*255.)
        dL = 0
        dR = 1


        if sL < 0:
            dL = 1
            sL = -sL

        if sR < 0:
            dR = 0
            sR = -sR 

        pub_msg = UInt8MultiArray() 
        pub_msg.data = [sL, sR, dL, dR] 
        self.pub.publish(pub_msg) 
        

	 


if __name__ == '__main__':
    
    try:
        teleop = TeleopDriver() 

    except rospy.ROSInterruptException:
        print ("Shutting down...")
