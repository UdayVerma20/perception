#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from perception.msg import pid_input
kp=1
kd=0
prev_error=0
scale = 1.15
pub= rospy.Publisher('MotorControl',Float32,queue_size=10)
def callback(data):
    global kp,kd,prev_error
    # error=-data.pid_error/1.5 + 1 
    error = data.pid_error/4 + 0.5 #{0,1}
    # pid error -> max displacement(1.5)+ max f*sin(a) (0.5) {-2,2}

    a=(kp*error + kd*(error-prev_error))#{-kd,kp+kd} 
    a = (a + kd)/(kp + 2*kd) # {0,1}
    # a = error
    a = ((a*800) + 100)
    if (a>500):
        a *= scale
    else:
        a /= scale
    print(a)
    
    prev_error=error
    motormsg = Float32()
    motormsg.data = a
    pub.publish(motormsg)

if __name__ == "__main__":
	print("Path_Planning")
	rospy.init_node('Control')
	rospy.Subscriber('/err', pid_input, callback)
	rospy.spin()