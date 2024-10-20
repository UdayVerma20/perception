#!/usr/bin/env python3
import rospy
import serial
from perception.msg import Coordinates


def Odometry():
    pub = rospy.Publisher('odometry', Coordinates, queue_size=10)
    rospy.init_node('Odometry', anonymous=True)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.reset_input_buffer()
    prev = 0
    while True:
        if ser.in_waiting > 0:
            line = int(ser.readline().decode('utf-8').rstrip())
            if line - prev > 0:
                Position.y += distance
                pub.publish(Position)
            prev = line


if __name__ == '__main__':
    # try:
    Position = Coordinates()
    Position.x = Position.y = Position.z = Position.colour = 0
    
    distance = (1/4) * (2 * 3.14 * 0.16)
    # Odometry()
    pub = rospy.Publisher('odometry', Coordinates, queue_size=10)
    rospy.init_node('Odometry', anonymous=True)
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    ser.reset_input_buffer()
    prev = 0
    i = 1
    while True:
        if ser.in_waiting > 0:
            # print("1")
            line = ser.readline().decode("utf-8", 'backslashreplace')
            curr = int((line == "1\r\n"))
            # print(curr)
            if curr - prev > 0:
                print(i)
                i+=1
                Position.y += distance
            pub.publish(Position)
            prev = curr
    # except rospy.ROSInterruptException:
    #     pass