#!/usr/bin/env python3
import rospy
import math
import numpy as np
from clustering.msg import CoordinateList
from clustering.msg import Coordinates
from perception.msg import uday
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from perception.msg import imu
# from datetime import datetime
# from rosgraph_msgs.msg import Clock
from tf.transformations import euler_from_quaternion, quaternion_from_euler

prev_yaw=0 
prev_m=0
prev_s=0
prev_ns=0
delta_t=0
yaw_t1=0
yaw=0
b=True
theta=0
a=True
cones=[]
orig_theta = 0

def Imucall(data):
	global roll, pitch, yaw,a, theta, yaw_t1, orig_theta
	orientation_q = data.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	# print("Y",yaw)
	yaw = math.degrees(yaw)
	print("yaw",yaw)
	
	if yaw<0:
		yaw +=360
	# print("Yaw",yaw)
	# print("OrigTheta",orig_theta)

	if(a):
		orig_theta = yaw
		# theta=yaw - orig_theta
		# yaw_t1=yaw - orig_theta
		a=False
	# print("Yaw ",yaw)
	# print("2",orig_theta)
	# t=(yaw-orig_theta)
	# print(t)
	theta =math.radians(yaw-orig_theta)
	print("theta", yaw-orig_theta)	

if __name__ == "__main__":
	print("SLAM")
	rospy.init_node('Slam')
	rospy.Subscriber("/imu/data", Imu, Imucall)
	rospy.spin()