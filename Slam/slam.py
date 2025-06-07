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

car_coordinate=[0,0]
min_distance=10000
min_left_coordinate=[-1,-1]
min_right_coordinate=[-1,-1]
left_cone_coordinates=[]
right_cone_coordinates=[]
pub= rospy.Publisher('/slam_to_distfinder',uday,queue_size=10)
car_coordinate_pub = rospy.Publisher('/CarCoordinate',Coordinates,queue_size=1)
imu_pub=rospy.Publisher('/Imu_yaw',imu,queue_size=1)
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
clus = True
cones=[]
orig_theta = 0

def distance(x,y):
	d=math.sqrt(((x[0]-y[0])**2)+((x[1]-y[1])**2))
	return d

def Imucall(data):
	global roll, pitch, yaw,a, theta, yaw_t1, orig_theta
	orientation_q = data.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	# print("Y",yaw)
	yaw = math.degrees(yaw)
	# print("1",yaw)
	
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
	# print("theta", yaw-orig_theta)
	message=imu()
	message.yaw=theta
	
	imu_pub.publish(message)
	

def call(data):
	global theta,car_coordinate,min_distance, min_left_coordinate,min_right_coordinate,left_cone_coordinates,right_cone_coordinates,b,cones, yaw, yaw_t1, orig_theta,cones
	# d=0.205
	if (clus):
		return
	d = data.data
	print("d=",d)
	print("theta=",math.degrees(theta))
	

	# print("theta",theta)
	# yaw_t1=yaw
	car_coordinate[0] += d*np.cos(theta)
	car_coordinate[1] += d*np.sin(theta)
	current_coordinate = Coordinates()
	current_coordinate.x = car_coordinate[0]
	current_coordinate.y = car_coordinate[1]
	current_coordinate.z = theta
	print("Current Coordinates ", current_coordinate)
	# current_coordinate.z = current_coordinate.color = 0
	car_coordinate_pub.publish(current_coordinate)
	# print(cones)
	for i in range(len(cones)):
		cones[i][0]+= car_coordinate[0]
		cones[i][1]+= car_coordinate[1]
	print("------------------------------")
	print("cones=",cones)
	
	# car_coordinate[1]+=d
	# print("car_x",car_coordinate[0])
	# print("car_y",car_coordinate[1])
	if(len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0):
		reference_x=car_coordinate[0]
		reference_y=car_coordinate[1]
	else:
		reference_x=(left_cone_coordinates[-1][0]+right_cone_coordinates[-1][0])/2
		reference_y=(left_cone_coordinates[-1][1]+right_cone_coordinates[-1][1])/2
	reference=[reference_x,reference_y]
	print("reference",reference)
	cones_new=[]
	flag=[1]*len(cones)
	if(len(left_cone_coordinates)!=0 and len(right_cone_coordinates)!=0):
		for i in range(0,len(cones)):
			for j in range(0,len(left_cone_coordinates)):
				p=[cones[i][0],cones[i][1]]
				# print("distleft",left_cone_coordinates[j],p,distance(left_cone_coordinates[j],p))
				# print("distright",right_cone_coordinates[j],p,distance(right_cone_coordinates[j],p))
				if(distance(left_cone_coordinates[j],p)<2 or distance(right_cone_coordinates[j],p)<2 ):
					# print("removed",p)
					flag[i]=0
					#cones_new.append(cones[i])

		for i in range(0,len(cones)):
			if(flag[i]==1):
				cones_new.append(cones[i])
		cones=cones_new
					
	# print("removed",cones)
		# print("cones_new",cones_new)
		# cones=cones_new
		#print("cones_new",cones)
	
	print("c",cones)
	for i in range(0,len(cones)):
		for j in range(i+1,len(cones)):
			mid_x=(cones[i][0]+cones[j][0])/2
			mid_y=(cones[i][1]+cones[j][1])/2
			mid=[mid_x,mid_y]
			minimum=distance(reference,mid)
			print("==")
			print("first cone=",cones[i])
			print("second cone=",cones[j])
			print("distance between them=",minimum)
			print("--")
			if(minimum<min_distance):
				min_distance=minimum
				#the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
				#be the position of right cone and cones[j] will ve the position of left cone
				l=[0,0]
				r=[0,0]
				l[0]=cones[j][0]
				l[1]=cones[j][1]
				r[0]=cones[i][0]
				r[1]=cones[i][1]
				min_left_coordinate=l
				min_right_coordinate=r
	print("min left coordinate=",min_left_coordinate)
	print("min right coordinate=",min_right_coordinate)

	if(len(left_cone_coordinates)==0 and distance(reference,car_coordinate)<1):
		left_cone_coordinates.append(min_left_coordinate)
		right_cone_coordinates.append(min_right_coordinate)
		message=uday()
		message.leftcone=min_left_coordinate
		message.rightcone=min_right_coordinate
		pub.publish(message)
		print("published")
		min_distance=10000
		min_left_coordinate=[-1,-1]
		min_right_coordinate=[-1,-1]
		
	else:
		A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
		B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
		C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
		d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
		# print("d", d)
		if(d<1):
			left_cone_coordinates.append(min_left_coordinate)
			right_cone_coordinates.append(min_right_coordinate)
			message=uday()
			message.leftcone=min_left_coordinate
			message.rightcone=min_right_coordinate
			pub.publish(message)
			print("published")
			min_distance=10000
			min_left_coordinate=[-1,-1]
			min_right_coordinate=[-1,-1]

	# print(left_cone_coordinates)
	# print(right_cone_coordinates)
	# print("----------------")


def callbackabx(data):
	global clus
	clus = False
	# print("hi")
	global cones
	cones = []
	for i in data.ConeCoordinates:
		cones.append([i.x, i.y])
	# print(cones)
	

if __name__ == "__main__":
	print("SLAM")
	rospy.init_node('Slam')
	rospy.Subscriber("/Clusters", CoordinateList, callbackabx)
	# rospy.Subscriber("/FusedCoordinates", CoordinateList, callbackabx)
	rospy.Subscriber("/distance_hall",Float32, call)
	rospy.Subscriber("/imu/data", Imu, Imucall)
	rospy.spin()