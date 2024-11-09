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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from perception.msg import landmark_points


# pub = rospy.Publisher('/final_coordinates', final_coordinates, queue_size=100)

STATE_SIZE = 3  # State size [x,y,yaw]
prev_m=0
prev_s=0
prev_ns=0
delta_t=0
bt=True
cones=[]
global landmark_indexes
landmark_indexes= 37
landmarks=[]
num_landmarks=0
prev_yaw=[0]
global u
u=[0.48,0]
global z
z=[]
global xEst
global PEst
prev_steer=0
steer=0
p=False
o=True
a=True
xEst = np.zeros((STATE_SIZE, 1))
orig_theta = 0

PEst = 1e-30 * np.full((3 + 2 * landmark_indexes, 3 + 2 * landmark_indexes), 1)
for i in range(3, 3 + 2 * landmark_indexes):
    PEst[i][i] = 1e-15                   #-1

difference_f=0
H_f = np.zeros((3, 3 + 2 * landmark_indexes))
Psi_f = np.zeros((3, 3))
sutta=[0,0]
Q = np.diagflat(np.array([0.08, 0.08, 0.08]))**2
car_coordinate=[0,0]


pub= rospy.Publisher('/landmark',landmark_points,queue_size=100)

def ekf_slam():

    global xEst,PEst,z,cones,car_coordinate
    for i in range(len(cones)):
        cones[i][2]= math.sqrt(((cones[i][0])*2)+((cones[i][1])*2))
        cones[i][3]=math.atan(cones[i][1]/cones[i][0])
        cones[i][0]+= car_coordinate[0]
        cones[i][1]+= car_coordinate[1]
    print("car",car_coordinate)
    # print(cones)  
    

    for i in range(len(cones)):
        cones[i][0]+= car_coordinate[0]
        cones[i][1]+= car_coordinate[1]

    predict()
    x=[]
    y=[]
    for i in cones:
        a=True
        for j in landmarks:
            if(math.sqrt(((j[0]-i[0])**2)+((j[1]-i[1])**2))<0.4):
                a=False
                break
        if(a):
            landmarks.append(i)
    print(landmarks)
    for i in landmarks:
        x.append(i[0])
        y.append(i[1])

    msg=landmark_points()
    msg.land_x=x
    msg.land_y=y
    pub.publish(msg)

        
    # for measurement in cones:
    #     #H_f,Psi_f,difference_f,K=data_association(xEst,PEst,measurement,z)
    #     temp_dataassociation(measurement)
    
   
    # msg=final_coordinates()
    # msg.x=xEst[0][0]
    # msg.y=xEst[1][0]rslidar
    # pub.publish(msg)
    # print("after")
    # print(xEst)


def predict():
    global xEst,PEst,delta_t, landmark_indexes
    
    print("starting")
    print(xEst)
    #print(PEst)
    
    print("------------------------------------------------------------------------")
    theta=xEst[2][0]
    print("theta=",theta)
    xEst[2][0]=theta
    x=xEst[0][0]+0.205*np.cos(xEst[2][0])
    y=xEst[1][0]+0.205*np.sin(xEst[2][0])
    #x=xEst[0][0]+0.2*np.cos(xEst[2][0])
    #y=xEst[1][0]+0.2*np.sin(xEst[2][0])
    '''
    if (theta > np.pi):
        theta -= 2 * np.pi

    elif (theta < -np.pi):
        theta += 2 * np.pi
    '''
    xEst[0][0]=x
    xEst[1][0]=y
    

    G = np.identity(3 + 2 * (landmark_indexes))
    G[0][2] = - 0.205 * np.sin(xEst[2][0])
    G[1][2] = 0.205 * np.cos(xEst[2][0])
    #G[0][2] = -0.2* np.sin(xEst[2][0])
    #G[1][2] = 0.2 * np.cos(xEst[2][0])

    sigma=np.dot(np.dot(G,PEst),(np.transpose(G)))
    PEst=sigma
    
    
    # print("---------------------------------------")
    print("after")
    print(xEst)
    


def temp_dataassociation(measurement):
    global landmarks,num_landmarks, landmark_indexes, PEst, Psi_f, difference_f, H_f, Q
    min=10
    j=0
    index=0
    for i in landmarks:
        dist=math.sqrt(((abs(i[1]-measurement[1]))*2) + ((abs(i[0]-measurement[0]))*2))
        if(dist<min):
            min=dist
            index=j
        j=j+1
    #print(landmarks[index])
    if ((index==0 and min>1.2) or (min>1)):
        #print("not stored")
        return
    x_t=xEst[0][0]
    y_t=xEst[1][0]
    theta_t=xEst[2][0]
    range_t=measurement[2]
    bearing_t=measurement[3]

    min_distance = 1e16

    cone_x=car_coordinate[0]+ range_t*math.cos(bearing_t)
    cone_y=car_coordinate[1]+ range_t*math.sin(bearing_t)

    x_l=landmarks[index][0]
    y_l= landmarks[index][1]
    delta_x = (cone_x - x_t)
    delta_y = (cone_y - y_t)
    q = delta_x * 2 + delta_y * 2
    range_expected = np.sqrt(q)
    bearing_expected = np.arctan2(delta_y, delta_x) #- theta_t
    #print("range=",range_t,range_expected,range_t-range_expected)
    #print("bearing=",math.degrees(bearing_t),math.degrees(bearing_expected),math.degrees(bearing_t)-math.degrees(bearing_expected))
    # print("----------------------------------------------------")
    # print(range_t-range_expected,math.degrees(bearing_t)-math.degrees(bearing_expected))
    
    #print("bearing=",bearing_expected,math.degrees(bearing_expected))

    F_x = np.zeros((5, 3 + 2 * (landmark_indexes)))
    F_x[0][0] = 1.0
    F_x[1][1] = 1.0
    F_x[2][2] = 1.0
    F_x[3][2 * index + 3] = 1.0
    F_x[4][2 * index + 4] = 1.0
    H_1 = np.array([-delta_x/np.sqrt(q), -delta_y/np.sqrt(q), 0, delta_x/np.sqrt(q), delta_y/np.sqrt(q)])
    H_2 = np.array([delta_y/q, -delta_x/q, -1, -delta_y/q, delta_x/q])
    H_3 = np.array([0, 0, 0, 0, 0])
    H_f = np.array([H_1, H_2, H_3]).dot(F_x)

    Psi_f = np.dot(np.dot(H_f,PEst),np.transpose(H_f))+ Q
    difference_f =  np.array([range_t-range_expected,bearing_t-bearing_expected,0])               #  np.array([range_t-range_expected, bearing_t-bearing_expected, 0])
    # if((math.degrees(bearing_t)-math.degrees(bearing_expected))>2):
    #    return
    #print(difference_f[0],difference_f[1])
    #if(abs(difference_f[0][0])>0.25):
    update()



def update():
    """
    Performs the update step of EKF SLAM

    :param xEst:  nx1 the predicted pose of the system and the pose of the landmarks
    :param PEst:  nxn the predicted covariance
    :param z:     the measurements read at new position
    :param initP: 2x2 an identity matrix acting as the initial covariance
    :returns:     the updated state and covariance for the system
    """
    global xEst, PEst, H_f,Psi_f, difference_f, landmark_indexes
    
    
    K = np.dot(np.dot(PEst,H_f.T),np.linalg.inv(Psi_f))      #PEst.dot(H_f.T).dot(np.linalg.inv(Psi_f)) 
    innovation = np.dot(K,difference_f)
    print("innovation=",innovation[0],innovation[1],innovation[2])
    if(abs(innovation[0])>1 or abs(innovation)[1]>1 ):
        return
        
    xEst[0][0]+=innovation[0]*(1)
    xEst[1][0]+=innovation[1]*(1)
    xEst[2][0]+=innovation[2]*(1)
    
    # Update covariance
    PEst = (np.identity(3 + 2 * (landmark_indexes)) - np.dot(np.dot(K,H_f),PEst))

    

        

def real(data):
    
    global roll, pitch, yaw,a, orig_theta
    orientation_q = data.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    yaw = math.degrees(yaw)
    # print("1",yaw)
	
    if yaw<0:
        yaw +=360
    if(a):
        orig_theta = yaw
        a=False
    # print("Yaw ",yaw)
    # print("2",orig_theta)
    yaw=(yaw-orig_theta) #t in degrees
    xEst[2][0]=yaw
    # print(yaw)

    


def callback(data):
    global cones
    cones = []
    for i in data.ConeCoordinates:
        cones.append([i.x, i.y,0,0])
    #     print(i)
    # print("------")
    # print(1)
    # global cones
    # global xEst,PEst,delta_t, z,num_landmarks, landmarks
    # global car_coordinate , sutta, bt, delta_t, prev_m, prev_s, prev_ns, prev_steer,prev_yaw,p,o
    # l=int(len(data.oned_list)/4)
    # d=(np.array(data.oned_list).reshape((l, 4)))
    # cones=d.tolist()
    # z=cones
    # for i in z:
    #     t=True
        
    #     for j in landmarks:
    #         d=math.sqrt(((abs(i[1]-j[1]))*2) + ((abs(i[0]-j[0]))*2))
    #         if(d<1):
    #             t=False
    #             break
    #     if(t):
    #         landmarks.append(i)
    #         num_landmarks+=1
    

def call(data):
    global p
    p=True
    

def halleffect(data):
    print("lulli")
    ekf_slam()



if __name__ == '__main__':
    print("Hokuyo LIDAR node started")
    rospy.init_node('rand',anonymous = True)
    rospy.Subscriber("/Clusters", CoordinateList, callback)
    rospy.Subscriber("/imu/data", Imu, real)
    # rospy.Subscriber("/controltoekf", ekf, call)
    rospy.Subscriber("/distance_hall",Float32, halleffect)
    
    
    
    rospy.spin()