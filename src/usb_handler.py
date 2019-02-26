#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
import serial.tools.list_ports
from Cabinet.srv import *
import time
# from arduino.srv import *

parrot_serial = None
arduino_serial = None
baudrates = 115200


def Search_for_parrot_and_arduino_serial_port():
    global parrot_serial,arduino_serial
    available_ports = [tuple(p) for p in list(serial.tools.list_ports.comports())]
    for port in available_ports:
        test_serial = serial.Serial(
            port = port[0],
            baudrate = baudrates
        )
        test_serial.close()
        test_serial.open()
        time_last = time.time()
        while True:
            if test_serial.inWaiting():
                text = test_serial.readline()#.decode('utf-8')
                if text.find('parrot') != -1:
                    parrot_serial = test_serial
                    print('!!!parrot port found on port %s!!!'%port[0])
                    break
                elif text.find('arduino') != -1:
                    arduino_serial = test_serial
                    print('!!!arduino port found on port %s!!!'%port[0])
                    break
                else:pass


            if time.time()-time_last > 10:
                print('!!!! this port is not arduino or parrot %s  !!!!'%port[0])
                break


def handle_parrot_connection(req):
    parrot_serial.write(req.command)
    print(req)
    return ParrotResponse(parrot_serial.readline().decode('utf-8'))


def handle_arduino_connection(req):
    arduino_serial.write(req.command)
    return ArduinoResponse("sent")
    # return ArduinoResponse(arduino_serial.readline().decode('utf-8'))

def handel_connections():
    rospy.init_node('usb_handler')
    parrot_service = rospy.Service('usb_handler/parrot', Parrot, handle_parrot_connection)
    parrot_service = rospy.Service('usb_handler/arduino', Arduino, handle_arduino_connection)
    rospy.spin()

if __name__ == "__main__":
    Search_for_parrot_and_arduino_serial_port()
    print("start the service")
    handel_connections()
