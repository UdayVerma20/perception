#!/usr/bin/env python3

# from tf2_msgs.msg import TFMessage
# import csv
# import itertools

import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import scipy
import numpy as np
from tf.transformations import euler_from_quaternion
from clustering.msg import Coordinates
from perception.msg import imu
from perception.msg import path_coordinates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from sensor_msgs.msg import Imu

x_tf = []
current_states = []
# i = 2
x_pos = 0.0
y_pos = 0.0
cur_yaw = 0.0
delta = 0.0

vel = 0.1
theta = 0
beta = 0
wh_ba = 1.75

prev_time = 0.0
prev_yaw = 0.0
time = 0.001
yaw=0
f=[0,0]
e=[0,0]
total = []

pub = rospy.Publisher('/MotorControl', Float32, queue_size=10)
marker_pub = rospy.Publisher("/control_viz", Marker, queue_size=100)
shape = Marker.LINE_STRIP
def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):
        return True
    else:
        return False
def Steeringcall(data):
    global delta
    delta = data.data

def motorcall(data):
    global vel
    vel = data.data


def Imucall(data):
	global curr_yaw
	orientation_q = data.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	curr_yaw = yaw

def cords(data):
    global f,e
    f=data.first
    print("f",f)
    e=data.second
    return f,e
    
def callback(data):
    global current_states, x_pos, y_pos,curr_yaw,delta
    x_pos=data.x
    y_pos=data.y
    current_states = np.array([[x_pos], [y_pos], [curr_yaw], [delta]])
    print(current_states)
    print(current_states[1][0])
    error()


def error():
    global current_states, x_pos, y_pos, cur_yaw, delta, vel, theta, beta, wh_ba, prev_yaw, prev_time, time, x_tf,i,heading ,total
    if(f[0] - 0.5 <= current_states[0][0] <= f[0] + 0.5 and f[1] - 0.5 <= current_states[1][0] <=f[1] + 0.5):
        heading = (f[1] - current_states[1][0])/(f[0] - current_states[0][0])
        x_tf = np.array([[f[0]],[f[1]],[heading],[0]])
        # total[0] = [f[0],f[1]]
        # total[1] = [current_states[0][0], current_states[1][0]]
        
    error_states = x_tf-current_states

    if(prev_time==0.0):
        prev_time = rospy.get_time()
        return
    cur_time = rospy.get_time()
    time = cur_time-prev_time
    if(withinthresh(time, 0.05)):
        return
    prev_time = cur_time

    R = np.array([[1,0],[0,1]])
    Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    A = np.array([[1,0,-vel*time*math.sin(cur_yaw),0],[0,1,vel*time*math.cos(cur_yaw),0],[0,0,1,(vel/wh_ba)*((1/math.cos(delta))*2)],[0,0,0,1]])
    B = np.array( [ [math.cos(cur_yaw)*time,0],[time*math.sin(cur_yaw),0],[(math.tan(delta)*time)/wh_ba,0],[0,time]] )

    P = scipy.linalg.solve_continuous_are(A,B,Q,R)
    S = np.dot(np.linalg.inv(R),np.transpose(B))
    K = np.dot(S,P)
    print(K)
    # print(U)
    U = np.dot(K,error_states)
    if(U[1][0]>55*3.141592653589/180):
        U[1][0]=55*3.141592653589/180
    if(U[1][0]<-65*3.141592653589/180):
        U[1][0]=-65*3.141592653589/180
    # print(U)
    U[0][0] *= 0.05
    print(U)
    print()
    control(U)


def control(input_array):
    command1 = Float32()
    command1.data = input_array[1][0]*180/math.pi
    pub.publish(command1)


if __name__ == '__main__':
    # global total
    print("bruh started")
    rospy.init_node('myLQR', anonymous = False)
    rospy.Subscriber("/CarCoordinate",Coordinates, callback)
    rospy.Subscriber("/RPM",Float32, motorcall)
    rospy.Subscriber("/SteeringPosition", Float32, Steeringcall)
    rospy.Subscriber("/imu/data", Imu, Imucall)
    rospy.Subscriber("/path",path_coordinates, cords)
    rospy.spin()
    # rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    #     marker = Marker()

    #     # Set the frame ID and timestamp
    #     marker.header.frame_id = "map"
    #     marker.header.stamp = rospy.Time.now()

    #     # Set the namespace and ID for this marker
    #     marker.ns = "basic"
    #     marker.id = 0

    #     # Set the marker type
    #     marker.type = shape

    #     # Set the marker action
    #     marker.action = Marker.ADD

    #     # Set the pose of the marker
        
    #     marker.pose.position.x = 0
    #     marker.pose.position.y = 0
    #     marker.pose.position.z = 0
    #     marker.pose.orientation.x = 0.0
    #     marker.pose.orientation.y = 0.0
    #     marker.pose.orientation.z = 0.0
    #     marker.pose.orientation.w = 1.0
        
    #     # Set the scale of the marker
    #     marker.scale.x = 0.05
    #     marker.scale.y = 0.0
    #     marker.scale.z = 0.0

    #     # Set the color
    #     marker.color.r = 0.0
    #     marker.color.g = 1.0
    #     marker.color.b = 0.0
    #     marker.color.a = 1.0

    #     # Define the points for the line strip
        
    #     # print("total=",total)
        
    #     for i in total:
    #         p=Point()
    #         p.x=i[0]
    #         p.y=i[1]
    #         p.z=0
    #         marker.points.append(p)
        

    #     # global car_coordinate 
    #     # p1 = Point()
    #     # p1.x=car_coordinate[0]
    #     # p1.y=car_coordinate[1]
    #     # p1.z=0

    #     # marker.points.append(p1)

        

       
    #     #marker.points=total
       
    #     marker_pub.publish(marker)
       

    #     # Cycle between different shapes

    #     rate.sleep()