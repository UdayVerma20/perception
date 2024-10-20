#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from race.msg import slam
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from race.msg import final_coordinates
from race.msg import pid_input



forward_projection = 0.5  #0.5	
vel = 15 		
error = 0.0		
car_length = 0.50 
b=True

leftcone=[]
rightcone=[]
car_coordinate=[0,0]

pub = rospy.Publisher('/err', pid_input, queue_size=10)





def mainloop():
	rospy.init_node('dist_finder',anonymous = True)
	rospy.Subscriber("/gt_pose",PoseStamped, callback)
	rospy.Subscriber("/slam_to_distfinder",slam,fall)
	rospy.Subscriber("/final_coordinates",final_coordinates, d)

	while not rospy.is_shutdown():
		midpoints=[]

	if(len(leftcone)==0 or len(rightcone)==0):
		leftcone.append(car_coordinate)
		rightcone.append(car_coordinate)
		
	if(len(leftcone)>=2):
		for i in range(2):
			mid=[]
			min_left= leftcone[i]
			min_right= rightcone[i]
			mid.append((min_left[0]+min_right[0])/2)
			mid.append((min_left[1]+min_right[1])/2)
			midpoints.append(mid)

		num=midpoints[1][1]-midpoints[0][1]
		den=(midpoints[1][0]-midpoints[0][0])

		slope= abs(num/den)
		ideal_orientation=math.atan(slope)

		dist_midpoints=abs(math.sqrt(num*2 + den*2))
		ideal=math.acos(den/dist_midpoints)
		
		val=midpoints[1][1]-midpoints[0][1]

		if(val<0):
			ideal=ideal*(-1)

		A=midpoints[0][1]-midpoints[1][1]
		B=midpoints[1][0]-midpoints[0][0]
		C=(midpoints[0][0]*midpoints[1][1])-(midpoints[1][0]*midpoints[0][1])
		err=(abs((A*car_coordinate[0])+(B*car_coordinate[1]+C))/math.sqrt((A*2)+(B*2)))

		dist_car_left= math.sqrt(((leftcone[1][0]-car_coordinate[0])*2)+((leftcone[1][1]-car_coordinate[1])*2))
		dist_car_right= math.sqrt(((rightcone[1][0]-car_coordinate[0])*2)+((rightcone[1][1]-car_coordinate[1])*2))

		Alpha= yaw-ideal
		if(dist_car_left<=dist_car_right):
			error=forward_projection*math.sin(Alpha)+err
		else:
			error=forward_projection*math.sin(Alpha)-err

		A1=leftcone[1][1]-rightcone[1][1]
		B1=rightcone[1][0]-leftcone[1][0]
		C1=(leftcone[1][0]*rightcone[1][1])-(rightcone[1][0]*leftcone[1][1])
		d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1*2)+(B1*2)))

		if(d>0 and d<0.2):
			leftcone.pop(0)
			rightcone.pop(0)

if _name_ == '_main_':
	try:
		mainloop()
	except rospy.ROSInterruptException:
		pass