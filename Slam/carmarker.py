#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
# from geometry_msgs.msg import PoseStamped
from clustering.msg import Coordinates
from time import sleep
import math
# from race.msg import final_coordinates
car_coordinate=[0,0]
prev=[0,0]
total=[]
pres=[]
diff=0
global_distance=0


def call(data):
    global car_coordinate 
    global diff
    global prev
    global pres
    car_coordinate=[0,0]
    car_coordinate[0]=data.x
    car_coordinate[1]=data.y

    #print("present",car_coordinate)
    #print("prev prev",prev)
    # diff=math.sqrt(((abs(car_coordinate[1]-prev[1]))*2) + ((abs(car_coordinate[0]-prev[0]))*2))
    
    total.append(car_coordinate)
    global global_distance
    if(len(total)>1):
        global_distance += ((total[-1][0]-total[-2][0])**2 + (total[-1][1]-total[-2][1])**2)**0.5
    # print("Global Distance ", global_distance)



    #print("prev",prev)
    "--------------------------------------------------------"
    

def main():
    rospy.init_node("CarMarker")
    marker_pub = rospy.Publisher("/track_viz", Marker, queue_size=1)
    rate = rospy.Rate(1)
    rospy.Subscriber("/CarCoordinate",Coordinates, call)
    # Set our initial shape type to be a cube
    shape = Marker.LINE_STRIP

    while not rospy.is_shutdown():
        marker = Marker()

        # Set the frame ID and timestamp
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # Set the namespace and ID for this marker
        marker.ns = "basic"
        marker.id = 0

        # Set the marker type
        marker.type = shape

        # Set the marker action
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.0)

        # Set the pose of the marker
        
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        marker.scale.x = 0.05
        marker.scale.y = 0.0
        marker.scale.z = 0.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Define the points for the line strip
        global total
        # print("total=",total)
        
        for i in total:
            p=Point()
            p.x=i[0]
            p.y=i[1]
            p.z=0
            marker.points.append(p)
        

        # global car_coordinate 
        # p1 = Point()
        # p1.x=car_coordinate[0]
        # p1.y=car_coordinate[1]
        # p1.z=0

        # marker.points.append(p1)

        

       
        #marker.points=total
       
        marker_pub.publish(marker)
       

        # Cycle between different shapes

        rate.sleep()

if __name__ == "__main__":
    total.append(car_coordinate)
    main()