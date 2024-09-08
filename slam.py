#!/usr/bin/env python3
import rospy
import math
import numpy as np
from perception.msg import CoordinateList
from perception.msg import Coordinates
from perception.msg import uday
from std_msgs.msg import Float32MultiArray

from geometry_msgs.msg import PoseStamped

car_coordinate=[0,0]
min_distance=10000
min_left_coordinate=[-1,-1]
min_right_coordinate=[-1,-1]
left_cone_coordinates=[]
right_cone_coordinates=[]
pub= rospy.Publisher('slam_to_distfinder',uday,queue_size=10)

def distance(x,y):
	d=math.sqrt(((x[0]-y[0])**2)+((x[1]-y[1])**2))
	return d

def call(data):
	global car_coordinate,min_distance, min_left_coordinate,min_right_coordinate,left_cone_coordinates,right_cone_coordinates
	car_coordinate[0]=data.x
	car_coordinate[1]=data.y


def callback(data):
	global car_coordinate,min_distance, min_left_coordinate,min_right_coordinate,left_cone_coordinates,right_cone_coordinates
	cones=data.ConeCoordinates
	if(len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0):
		reference_x=car_coordinate[0]
		reference_y=car_coordinate[1]
	else:
		reference_x=(left_cone_coordinates[-1][0]+right_cone_coordinates[-1][0])/2
		reference_y=(left_cone_coordinates[-1][1]+right_cone_coordinates[-1][1])/2
	reference=[reference_x,reference_y]

	if(len(left_cone_coordinates)!=0 and len(right_cone_coordinates)!=0):
		for j in range(0,len(left_cone_coordinates)):
			for i in cones:
				p=[i.x,i.y]
				if(distance(left_cone_coordinates[j],p)<1 or distance(right_cone_coordinates[j],p)<1):
					cones.remove(i)

	for i in range(0,len(cones)):
		for j in range(i+1,len(cones)):
			mid_x=(cones[i].x+cones[j].x)/2
			mid_y=(cones[i].y+cones[j].y)/2
			mid=[mid_x,mid_y]
			minimum=distance(reference,mid)
			# print("first cone=",cones[i])
			# print("second cone=",cones[j])
			# print("distance between them=",minimum)
			if(minimum<min_distance):
				min_distance=minimum
				#the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
				#be the position of right cone and cones[j] will ve the position of left cone
				l=[0,0]
				r=[0,0]
				l[0]=cones[j].x
				l[1]=cones[j].y
				r[0]=cones[i].x
				r[1]=cones[i].y
				min_left_coordinate=l
				min_right_coordinate=r
				# print("min left coordinate=",min_left_coordinate)
				# print("min right coordinate=",min_right_coordinate)

	if(len(left_cone_coordinates)==0 and distance(reference,car_coordinate)<0.4):
		left_cone_coordinates.append(min_left_coordinate)
		right_cone_coordinates.append(min_right_coordinate)
		message=uday()
		message.leftcone=min_left_coordinate
		message.rightcone=min_right_coordinate
		pub.publish(message)

		min_distance=10000
		min_left_coordinate=[-1,-1]
		min_right_coordinate=[-1,-1]
		
	else:
		A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
		B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
		C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
		d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
		if(d<0.4):
			left_cone_coordinates.append(min_left_coordinate)
			right_cone_coordinates.append(min_right_coordinate)
			message=uday()
			message.leftcone=min_left_coordinate
			message.rightcone=min_right_coordinate
			pub.publish(message)

			min_distance=10000
			min_left_coordinate=[-1,-1]
			min_right_coordinate=[-1,-1]
	print(left_cone_coordinates)
	print(right_cone_coordinates)
	print("----------------")

if __name__ == '__main__':
	print("SLAM")
	rospy.init_node('slam',anonymous = True)
	rospy.Subscriber("/Clusters",CoordinateList,callback)
	rospy.Subscriber("/odometry",Coordinates, call)
	rospy.spin()