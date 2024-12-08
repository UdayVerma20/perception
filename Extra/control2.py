#!/usr/bin/env python3
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import scipy
import numpy as np
from clustering.msg import Coordinates
from perception.msg import imu
from perception.msg import path_coordinates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

global ideal_state, current_state, cur_x, cur_y, cur_yaw, delta, vel, wh_ba, prev_time, prev_yaw, time, yaw, f, e, total, U, ideal_yaw, datareceived
ideal_state = []
current_state = []
cur_x = 0.0
cur_y = 0.0
cur_yaw = 0.0
delta = 0.0
vel = 0.1
wh_ba = 1.75
prev_time = 0.0
prev_yaw = 0.0
time = 0.001
yaw=0
f=[0,0]
e=[0,0]
total = [[0,0],[0,0]]
U = np.array([[0],[0]])
ideal_yaw=0
datareceived = [0,0,0,0]

pub = rospy.Publisher('/MotorControl', Float32, queue_size=10)
marker_pub = rospy.Publisher("/control_viz", Marker, queue_size=100)
shape = Marker.LINE_STRIP

def withinthresh(x, thresh):
    if(x>-thresh and x<thresh):
        return True
    else:
        return False
       
def steeringCallback(angledata):
    global delta, datareceived
    datareceived[0] = 1
    delta = angledata.data

def velCallback(velocitydata):
    global vel, datareceived
    datareceived[1] = 1
    vel = velocitydata.data

def imuCallback(yawdata):
    global cur_yaw, datareceived
    datareceived[2] = 1
    cur_yaw = yawdata.yaw

def pathplanningCallback(ideal_coord):
    global f, e, datareceived
    datareceived[3] = 1
    f=ideal_coord.first
    e=ideal_coord.second
    return f,e
   
def slamCallback(slam_coord):
    global ideal_state, current_state, cur_x, cur_y, cur_yaw, delta, vel, wh_ba, prev_time, prev_yaw, time, yaw, f, e, total, U, ideal_yaw, datareceived
    if(datareceived[0]!=1 or datareceived[1]!=1 or datareceived[2]!=1 or datareceived[3]!=1):
        return
    cur_x=slam_coord.x
    cur_y=slam_coord.y
    current_state = np.array([[cur_x], [cur_y], [cur_yaw], [delta]])
    compute()


def compute():
    global ideal_state, current_state, cur_x, cur_y, cur_yaw, delta, vel, wh_ba, prev_time, prev_yaw, time, yaw, f, e, total, U, ideal_yaw, datareceived
   
    #----------------time calculation---------------------
    if(prev_time==0.0):
        prev_time = rospy.get_time()
        return
    cur_time = rospy.get_time()
    time = cur_time-prev_time
    if(withinthresh(time, 0.05)):
        return
    prev_time = cur_time

    ideal_yaw = np.arctan2([e[1] - f[1]], [e[0] - f[0]])[0]
    ideal_state = np.array([[e[0]],[e[1]],[ideal_yaw],[0]])
    print("Ideal: ",ideal_state)
    print("Current: ",current_state)
    error_state = ideal_state-current_state

    total[0] = [e[0],e[1]]
    total[1] = [current_state[0][0], current_state[1][0]]

    R = np.array([[1,0],[0,1]])
    Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    # A = np.array([[1,0,-vel*math.sin(cur_yaw)*time,                                        0],
    #               [0,1, vel*math.cos(cur_yaw)*time,                                        0],
    #               [0,0,                          1,(vel/wh_ba)*((1/math.cos(delta))**2)*time],
    #               [0,0,                          0,                                        1]])

    # B = np.array( [ [      math.cos(cur_yaw)*time,   0],
    #                 [      time*math.sin(cur_yaw),   0],
    #                 [(math.tan(delta)*time)/wh_ba,   0],
    #                 [                           0,time]] )
   
    A = np.array([[0,0,   0,          0],
                  [0,0, vel,          0],
                  [0,0,   0,(vel/wh_ba)],
                  [0,0,   0,          0]])
   
    B = np.array( [ [          1,          0],
                    [          0,          0],
                    [delta/wh_ba,          0],
                    [          0,          1]] )

    P = scipy.linalg.solve_continuous_are(A,B,Q,R)
    S = np.dot(np.linalg.inv(R),np.transpose(B))
    K = np.dot(S,P)
    U = -1*np.dot(K,error_state)
    # U = [[VELOCITY],[STEERING]]

    if(U[1][0]>55*3.141592653589/180):
        U[1][0]=55*3.141592653589/180
    if(U[1][0]<-65*3.141592653589/180):
        U[1][0]=-65*3.141592653589/180
    U[0][0] *= 0.05
    print("U ",U)
    print()
    # control(U)


def control(input_array):
    command1 = Float32()
    command1.data = input_array[1][0]*180/math.pi      
    pub.publish(command1)


if __name__ == '__main__':
    # global total
    print("LQR")
    rospy.init_node('LQR', anonymous = False)
    rospy.Subscriber("/RPM", Float32, velCallback)
    rospy.Subscriber("/SteeringPosition", Float32, steeringCallback)
    rospy.Subscriber("/Imu_yaw", imu, imuCallback)
    rospy.Subscriber("/path", path_coordinates, pathplanningCallback)
    rospy.Subscriber("/CarCoordinate", Coordinates, slamCallback)
    # rospy.spin()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        marker = Marker()

        # Set the frame ID and timestamp
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "basic"
        marker.id = 0
        marker.type = shape
        marker.action = Marker.ADD

        # Set the pose of the marker
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for i in total:
            p=Point()
            p.x=i[0]
            p.y=i[1]
            p.z=0
            marker.points.append(p)
        marker_pub.publish(marker)
   
        rate.sleep()