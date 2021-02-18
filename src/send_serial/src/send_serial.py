#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

import sys
import serial
import time
import glob

def list_serial_ports():
    """ Lists serial port names
        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result

port_list = list_serial_ports()
print(port_list)

ser = serial.Serial()

ser.port = port_list[0]

ser.baudrate = 115200

ser.close()
ser.open()

time.sleep(2)


print("Serial Ready")

rospy.init_node("Send_serial")

def callback(msg):
    v = round(msg.linear.x*100,2)
    w = round(msg.angular.z*100,2)

    send_value = str(v) + ',' + str(w)

    ser.write(send_value.encode())

    print(v)
    print(w)

sub = rospy.Subscriber("/cmd_vel", Twist, callback)

rate = rospy.Rate(2)               # We create a Rate object of 2Hz

while not rospy.is_shutdown():     # Endless loop until Ctrl + C
    rate.sleep()

