#!/usr/bin/env python
import rospy
import serial
import time
import threading
from Cabinet.srv import *
from std_msgs.msg import String



def arduino_client(command):
    rospy.wait_for_service('usb_handler/arduino')
    try:
        arduino_connection = rospy.ServiceProxy('usb_handler/arduino', Arduino)
        result = arduino_connection(command)
        print(result)
        return result.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





def arduino_commands(data):
    # print("get here")
    if(int(data.data) == 90 ):   # trun lightwheel on
        arduino_client('5')
    elif(int(data.data) == 91 ):  # trun lightwheel on
        arduino_client('6')

def ros_init():
    rospy.init_node('arduino')
    rospy.Subscriber('web/parrot_commands', String, arduino_commands)
    rospy.spin()


if __name__ == "__main__":
        ros_init()
