#!/usr/bin/env python
import rospy
import serial
import time
import threading
from Cabinet.srv import *
from std_msgs.msg import String



play_sound = None
stop_sound = None


def parrot_client(command):
    rospy.wait_for_service('usb_handler/parrot')
    try:
        parrot_connection = rospy.ServiceProxy('usb_handler/parrot', Parrot)
        result = parrot_connection(command)
        return result.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def dance(number = 1):
    res = parrot_client('G1 S%d'%number)
    if(res.find("DONE") != 1):
        print ("done")
    else:
        print("not done")

def blink(pwm = 160):
    res = parrot_client('G2 S%d'%pwm)
    if(res.find("DONE") != 1):
        print ("done")
    else:
        print("not done")

def mouth(pwm): # open and close the mouth 255 -> open mouth      0 -> close mouth
    res = parrot_client('G3 S%d'%pwm)
    if(res.find("DONE") != 1):
        print ("done")
    else:
        print("not done")



def parrot_voice_commands(data):
    # TODO: tedad dahan zadan
    play_sound.publish(data.data)
    mouth(255)
    time.sleep(0.05)
    mouth(0)

def parrot_commands(data):
    if(int(data.data) == 0 ):   # dance
        dance()
    elif(int(data.data) == 1 ):  # blink
        blink()
    elif(int(data.data) == 2 ): # open mouth
        mouth(255)
    elif(int(data.data) == 3 ): # clode mouth
        mouth(0)


def ros_init():
    global play_sound,stop_sound
    rospy.init_node('blue_parrot')
    play_sound = rospy.Publisher('audio_player/play_sound', String, queue_size=10)
    stop_sound = rospy.Publisher('audio_player/stop_sound', String, queue_size=10)
    rospy.Subscriber('web/parrot_commands', String, parrot_commands)
    rospy.Subscriber("web/parrot_voice_commands", String, parrot_voice_commands)
    rospy.spin()


if __name__ == "__main__":
        ros_init()
