# #!/usr/bin/env python3
# # import rospy
# # import math
# # import numpy as np
# # from std_msgs.msg import Float32
# # from sensor_msgs.msg import Imu

# import serial
# import csv
# import time

# from tf.transformations import euler_from_quaternion, quaternion_from_euler

# prev_time = None
# vel_z = 0.0
# disp_z = 0.0

# fl = -1
# fr = -1
# roll, pitch, yaw = 0, 0, 0
# lin_acc_x, lin_acc_y, lin_acc_z = 0, 0, 0
# ang_vel_x, ang_vel_y, ang_vel_z = 0, 0, 0
# filename = ""

# def Imucall(data):
#     global roll, pitch, yaw
#     global lin_acc_x, lin_acc_y, lin_acc_z
#     global ang_vel_x, ang_vel_y, ang_vel_z
#     global fl, fr, filename
#     global prev_time, vel_z, disp_z

#     # Orientation
#     orientation_q = data.orientation
#     orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
#     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
#     roll = math.degrees(roll)
#     pitch = math.degrees(pitch)
#     yaw = math.degrees(yaw)

#     # Acceleration
#     lin_acc_x = data.linear_acceleration.x
#     lin_acc_y = data.linear_acceleration.y
#     lin_acc_z = data.linear_acceleration.z

#     # Angular velocity
#     ang_vel_x = data.angular_velocity.x
#     ang_vel_y = data.angular_velocity.y
#     ang_vel_z = data.angular_velocity.z    

#     # Integrate to find displacement in Z
#     current_time_ros = data.header.stamp.to_sec()
#     if prev_time is not None:
#         dt = current_time_ros - prev_time
#         vel_z += lin_acc_z * dt
#         disp_z += vel_z * dt
#     prev_time = current_time_ros

#     if fl == -1:
#         return 

#     t = time.localtime()
#     current_time = time.strftime("%H:%M:%S", t)
#     list_pi = [current_time, fl, fr, roll, pitch, yaw, lin_acc_x, lin_acc_y, lin_acc_z,
#                ang_vel_x, ang_vel_y, ang_vel_z, vel_z, disp_z]
    
#     with open(filename, "a+") as file:
#         writer = csv.writer(file)
#         writer.writerow(list_pi)


# def call(data):
#     global fl, fr
#     fl = data.data // 1000
#     fr = data.data % 1000

# if __name__ == "__main__":
#     rospy.init_node('Log')
#     rospy.Subscriber("/abc", Float32, call)
#     rospy.Subscriber("/imu/data", Imu, Imucall)

#     t = time.localtime()
#     current_time = time.strftime("%d_%m_%H-%M-%S", t)
#     filename = "/home/uday/catkin_ws/src/perception/Slam/DataLog/Log_" + current_time + ".csv"

#     with open(filename, "a+") as file:
#         writer = csv.writer(file)
#         writer.writerow(["current_time", "fl", "fr", "roll", "pitch", "yaw", 
#                          "lin_acc_x", "lin_acc_y", "lin_acc_z", 
#                          "ang_vel_x", "ang_vel_y", "ang_vel_z","disp_z"])

#     rospy.spin()

#!/usr/bin/env python3
import serial
import csv
import time

def list_binary(num):
    binary = bin(num).replace("0b", "")
    list_bin = list(binary)
    list_binary = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    for i in range(0,len(list_bin)):
        list_binary[len(list_binary)-len(list_bin)+i] = int(list_bin[i])
    return list_binary


if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
    ser.reset_input_buffer()
    t = time.localtime()
    current_time=time.strftime("%d-%m%H:%M:%S", t)
    filename = "/home/uday/catkin_ws/src/perception/Slam/DataLog/Log" + current_time + ".csv"
    file = open(filename,"a+")
    writer = csv.writer(file)
    writer.writerow(["current_time", "fl", "fr"])    
    file.close()
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            list_ard = line.split()
            t = time.localtime()
            current_time=time.strftime("%H:%M:%S", t)
            list_pi=[current_time,list_ard[0],list_ard[1]]
            file = open(filename,"a+")
            writer = csv.writer(file)
            writer.writerow(list_pi)
            file.close()
            print(list_pi)