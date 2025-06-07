#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32
from perception.msg import pid_input
kp=1
kd=0
prev_error=0
scale = 1.2
pub= rospy.Publisher('MotorControl',Float32,queue_size=10)

SteeringCentre = 400
SteeringMax = 1000
SteeringMin = 100 
SteeringVal = -1
forward_projection = 3

def Steeringcallback(data):
    global SteeringVal
    SteeringVal = data.data

def callback(data):
    global kp,kd,prev_error,SteeringVal
    # error=-data.pid_error/1.5 + 1 
    # error = 
    error = data.pid_error/((1.5+forward_projection)) + 0.5 #{0,1}
    # error = -error+1
    # pid error -> max displacement(1.5)+ max f*sin(a) (0.5) {-2,2}

    a=(kp*error + kd*(error-prev_error))#{-kd,kp+kd} 
    a = (a + kd)/(kp + 2*kd) # {0,1}
    # a = error
    if (a<=0.5):
        a = (a*(SteeringCentre-SteeringMin)/0.5 + SteeringMin)
    else:
        a = ((a-0.5)*(SteeringMax-SteeringCentre)/0.5 + SteeringCentre)
    a = min(SteeringMax,a)
    a = max(SteeringMin,a)
    # # a = ((a*450) + 100)
    # if (a>SteeringCentre):
    #     a *= scale
    #     a = min(a,SteeringMax)
    # else:
    #     a /= scale
    #     a = max(a,SteeringMin)
    
    print(a)
    print(SteeringVal)
    print()
    
    prev_error=error
    motormsg = Float32()
    motormsg.data = a
    pub.publish(motormsg)

if __name__ == "__main__":
    print("Controls")
    rospy.init_node('Control')
    rospy.Subscriber('/err', pid_input, callback)
    rospy.Subscriber('/Steering', Float32, Steeringcallback)
    rospy.spin()