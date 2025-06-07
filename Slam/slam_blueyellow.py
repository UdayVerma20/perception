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
cones_blue=[]
cones_yellow=[]
orig_theta = 0
prev_left = [0,1.5]
prev_right = [0, -1.5]
missing_projection_scale = 0.75
import numpy as np

'''def  (prev_blue, prev_yellow, new_blue, new_yellow):
    def midpoint(p1, p2):
        """Calculate the midpoint between two points."""
        return [(p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2]

    def vector(p1, p2):
        """Calculate vector from p1 to p2."""
        return [p2[0] - p1[0], p2[1] - p1[1]]

    def add_vector(p, v, scale=1.0):
        """Move point p by vector v scaled by 'scale'."""
        return [p[0] + v[0] * scale, p[1] + v[1] * scale]

    if new_blue == [-1, -1]:  # Blue cone is missing
        # Vector from previous blue to yellow
        v_by = vector(prev_blue, prev_yellow)
        # Half vector to maintain relative distance
        v_estimated_by = [v_by[0]*missing_projection_scale, v_by[1]*missing_projection_scale]
        # Estimate blue as a parallelogram completion
        new_blue = add_vector(new_yellow, [-v_estimated_by[0], -v_estimated_by[1]])

    elif new_yellow == [-1, -1]:  # Yellow cone is missing
        # Vector from previous yellow to blue
        v_yb = vector(prev_yellow, prev_blue)
        # Half vector to maintain relative distance
        v_estimated_yb = [v_yb[0]*missing_projection_scale, v_yb[1]*missing_projection_scale]
        # Estimate yellow as a parallelogram completion
        new_yellow = add_vector(new_blue, [-v_estimated_yb[0], -v_estimated_yb[1]])

    return new_blue, new_yellow'''

def estimate_missing_cone(prev_blue, prev_yellow, new_blue, new_yellow):
    print(new_blue," ",new_yellow)
    def vector(p1, p2):
        """Calculate vector from p1 to p2."""
        return [p2[0] - p1[0], p2[1] - p1[1]]

    def add_vector(p, v, scale=1.0):
        """Move point p by vector v scaled by 'scale'."""
        return [p[0] + v[0] * scale, p[1] + v[1] * scale]

    if new_blue == [-1,-1] :#and new_yellow!=[-1,-1]:  # Blue cone is missing
        # Vector from previous yellow to new yellow
        v_yy = vector(prev_yellow, new_yellow)
        # Half of that vector
        half_v_yy = [v_yy[0] *missing_projection_scale, v_yy[1] *missing_projection_scale]
        # Estimate blue by adding half vector to previous blue
        new_blue = add_vector(prev_blue, half_v_yy)
        print("new blue", new_blue)

    elif new_yellow == [-1,-1] :#and new_blue!=[-1,-1]:  # Yellow cone is missing
        # Vector from previous blue to new blue
        v_bb = vector(prev_blue, new_blue)
        # Half of that vector
        half_v_bb = [v_bb[0] *missing_projection_scale, v_bb[1] *missing_projection_scale]
        # Estimate yellow by adding half vector to previous yellow
        new_yellow = add_vector(prev_yellow, half_v_bb)
        print("new yellow", new_yellow)
    
    # elif new_blue == [-1,-1] and new_yellow==[-1,-1]:
    #     new_blue = add_vector(prev_blue, [half_v_yy])
    print("Estimate")
    print()
    return new_blue, new_yellow

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
    global theta,car_coordinate,min_distance, min_left_coordinate,min_right_coordinate,left_cone_coordinates,right_cone_coordinates,b,cones, yaw, yaw_t1, orig_theta,cones,cones_blue,cones_yellow, prev_right, prev_left
    # d=0.205
    if (clus):
        return
    d = data.data*0.9
    # print("d=",d)
    # print("theta=",math.degrees(theta))


    # print("theta",theta)
    # yaw_t1=yaw
    car_coordinate[0] += d*np.cos(theta)
    car_coordinate[1] += d*np.sin(theta)
    current_coordinate = Coordinates()
    current_coordinate.x = car_coordinate[0]
    current_coordinate.y = car_coordinate[1]
    current_coordinate.z = theta
    # print("Current Coordinates ", current_coordinate)
    # current_coordinate.z = current_coordinate.color = 0
    car_coordinate_pub.publish(current_coordinate)
    # print(cones)
    for i in range(len(cones_blue)):
        temp_x = cones_blue[i][0]
        temp_y = cones_blue[i][1]
        cones_blue[i][0] = temp_x*np.cos(theta) - temp_y*np.sin(theta)
        cones_blue[i][1] = temp_x*np.sin(theta) + temp_y*np.cos(theta)
        cones_blue[i][0] += car_coordinate[0]
        cones_blue[i][1] += car_coordinate[1]


    for i in range(len(cones_yellow)):
        temp_x = cones_yellow[i][0]
        temp_y = cones_yellow[i][1]          
        cones_yellow[i][0] = temp_x*np.cos(theta) - temp_y*np.sin(theta)
        cones_yellow[i][1] = temp_x*np.sin(theta) + temp_y*np.cos(theta)
        cones_yellow[i][0] += car_coordinate[0]
        cones_yellow[i][1] += car_coordinate[1]
    # print("------------------------------")
    # print("cones_blue=",cones_blue)
    # print("cones_yellow=",cones_yellow)
	
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
    # print("reference",reference)
    cones_new=[]
    flag=[1]*len(cones_blue)
    if(len(left_cone_coordinates)!=0):
        for i in range(0,len(cones_blue)):
            for j in range(0,len(left_cone_coordinates)):
                p=[cones_blue[i][0],cones_blue[i][1]]
                # print("distleft",left_cone_coordinates[j],p,distance(left_cone_coordinates[j],p))
                # print("distright",right_cone_coordinates[j],p,distance(right_cone_coordinates[j],p))
                if(distance(left_cone_coordinates[j],p)<2):
                    # print("removed",p)
                    flag[i]=0
                    #cones_blue_new.append(cones[i])

        for i in range(0,len(cones_blue)):
            if(flag[i]==1):
                cones_new.append(cones_blue[i])
        cones_blue=cones_new

    cones_new=[]
    flag=[1]*len(cones_yellow)
    if(len(right_cone_coordinates)!=0):
        for i in range(0,len(cones_yellow)):
            for j in range(0,len(right_cone_coordinates)):
                p=[cones_yellow[i][0],cones_yellow[i][1]]
                # print("distright",right_cone_coordinates[j],p,distance(right_cone_coordinates[j],p))
                # print("distright",right_cone_coordinates[j],p,distance(right_cone_coordinates[j],p))
                if(distance(right_cone_coordinates[j],p)<2):
                    # print("removed",p)
                    flag[i]=0
                    #cones_yellow_new.append(cones[i])

        for i in range(0,len(cones_yellow)):
            if(flag[i]==1):
                cones_new.append(cones_yellow[i])
        cones_yellow=cones_new
                    
	# print("removed",cones)
		# print("cones_new",cones_new)
		# cones=cones_new
		#print("cones_new",cones)
    if (len(cones_blue)>0 and len(cones_yellow)>0):
    # print("c",cones)
        for i in range(0,len(cones_blue)):
            for j in range(0,len(cones_yellow)):
                mid_x=(cones_blue[i][0]+cones_yellow[j][0])/2
                mid_y=(cones_blue[i][1]+cones_yellow[j][1])/2
                mid=[mid_x,mid_y]
                minimum=distance(reference,mid)
                # print("==")
                # print("first cone=",cones_blue[i])
                # print("second cone=",cones_yellow[j])
                # print("distance between them=",minimum)
                # print("--")
                if(minimum<min_distance):
                    min_distance=minimum
                    #the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
                    #be the position of right cone and cones[j] will ve the position of left cone
                    l=[0,0]
                    r=[0,0]
                    l[0]=cones_blue[i][0]
                    l[1]=cones_blue[i][1]
                    r[0]=cones_yellow[j][0]
                    r[1]=cones_yellow[j][1]
                    min_left_coordinate=l
                    min_right_coordinate=r
    elif (len(cones_blue)==0):
        for j in range(0,len(cones_yellow)):
            minimum=distance(reference,cones_yellow[j])
            # print("==")
            # print("first cone=",cones_blue[i])
            # print("second cone=",cones_yellow[j])
            # print("distance between them=",minimum)
            # print("--")
            if(minimum<min_distance):
                min_distance=minimum
                #the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
                #be the position of right cone and cones[j] will ve the position of left cone
                min_right_coordinate=cones_yellow[j]
        # min_right_coordinate = cones_yellow[0]
    else :
        for j in range(0,len(cones_blue)):
            minimum=distance(reference,cones_blue[j])
            # print("==")
            # print("first cone=",cones_blue[i])
            # print("second cone=",cones_blue[j])
            # print("distance between them=",minimum)
            # print("--")
            if(minimum<min_distance):
                min_distance=minimum
                #the coordinates of cones recieved are in order in which they are scaned by the lidar from right to left so when distance is observed cones[i] will
                #be the position of right cone and cones[j] will ve the position of left cone
                min_left_coordinate=cones_blue[j]
        # min_left_coordinate = cones_blue[0]
    # print("min left coordinate=",min_left_coordinate)
    # print("min right coordinate=",min_right_coordinate)

    if((len(left_cone_coordinates)==0 or len(right_cone_coordinates)==0) and distance(reference,car_coordinate)<1):
        min_left_coordinate, min_right_coordinate =  estimate_missing_cone(prev_left, prev_right, min_left_coordinate, min_right_coordinate)
        prev_left = min_left_coordinate
        prev_right = min_right_coordinate
        left_cone_coordinates.append(min_left_coordinate)
        right_cone_coordinates.append(min_right_coordinate)
        message=uday()
        message.leftcone=min_left_coordinate
        message.rightcone=min_right_coordinate
        pub.publish(message)
        # print("published")
        min_distance=10000
        min_left_coordinate=[-1,-1]
        min_right_coordinate=[-1,-1]
        
    else:
        A1=left_cone_coordinates[-1][1]-right_cone_coordinates[-1][1]
        B1=right_cone_coordinates[-1][0]-left_cone_coordinates[-1][0]
        C1=(left_cone_coordinates[-1][0]*right_cone_coordinates[-1][1])-(right_cone_coordinates[-1][0]*left_cone_coordinates[-1][1])
        d=(abs((A1*car_coordinate[0])+(B1*car_coordinate[1]+C1))/math.sqrt((A1**2)+(B1**2)))
        # print("d", d)
        if (d<1):
            print(prev_left,prev_right, min_left_coordinate, min_right_coordinate)
            min_left_coordinate, min_right_coordinate =  estimate_missing_cone(prev_left, prev_right, min_left_coordinate, min_right_coordinate)
            # if (min left or min right [-1-1]) create ||gm and replace
            prev_left = min_left_coordinate
            prev_right = min_right_coordinate
            left_cone_coordinates.append(min_left_coordinate)
            right_cone_coordinates.append(min_right_coordinate)
            message=uday()
            message.leftcone=min_left_coordinate
            message.rightcone=min_right_coordinate
            pub.publish(message)
            # print("published")
            min_distance=10000
            min_left_coordinate=[-1,-1]
            min_right_coordinate=[-1,-1]

    # print(left_cone_coordinates)
    # print(right_cone_coordinates)
    # print("----------------")


def callbackabx(data):
    global clus,cones_blue,cones_yellow
    clus = False
    # print("hi")
    cones_blue = []
    cones_yellow=[]
    for i in data.ConeCoordinates:
        if(i.colour==1):
            cones_blue.append([i.x, i.y])
        elif(i.colour==0):
            cones_yellow.append([i.x, i.y])


    # print(cones)
	

if __name__ == "__main__":
    print("SLAM")
    rospy.init_node('Slam')
    rospy.Subscriber("/Clusters", CoordinateList, callbackabx)
    # rospy.Subscriber("/FusedCoordinates", CoordinateList, callbackabx)
    rospy.Subscriber("/distance_hall",Float32, call)
    rospy.Subscriber("/imu/data", Imu, Imucall)
    rospy.spin()