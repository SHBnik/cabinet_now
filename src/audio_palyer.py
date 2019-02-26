#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import pygame
import time


def play_sound(data):
    pygame.mixer.Sound.play(pygame.mixer.Sound(data.data))

def stop_sound(data):
    pygame.mixer.stop()


def ros_init():
    rospy.init_node('audio_player')
    rospy.Subscriber("audio_player/play_sound", String, play_sound)
    rospy.Subscriber("audio_player/stop_sound", String, stop_sound)

def pygame_init():
    pygame.mixer.init()
    if pygame.mixer.get_init() is None:
        print("mixer initialization is NOT successful")



if __name__ == "__main__":
    ros_init()
    pygame_init()
    rospy.spin()
