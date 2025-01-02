#!/usr/bin/env python3
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import open3d.core as o3c
from open3d_ros_helper import open3d_ros_helper as orh
import numpy as np
import matplotlib.pyplot as plt
# import copy
# import os
# import sys
def callback(data):
    o3dpc = orh.rospc_to_o3dpc(data)
    # print(o3dpc)
    # print(np.asarray(o3dpc.points))  # Check the numpy array
    pcd = o3d.t.geometry.PointCloud(o3c.Tensor(np.asarray(o3dpc.points, dtype = np.float32), o3c.float32, o3c.Device("cuda:0")))
    # pcd = o3d.geometry.PointCloud(o3c.Tensor(np.asarray(o3dpc.points, dtype=np.float32), dtype=o3c.float32))#, device=o3c.Device("cuda:0")))
    # points = np.asarray(o3dpc.points, dtype=np.float32)  # Ensure points are in NumPy array format
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(points)
    # print(f"Number of points in the point cloud: {len(pcd.point.positions)}")
    # print(len(pcd.points))
    # print(np.asarray(pcd.points))
    labels = pcd.cluster_dbscan(eps=1, min_points=10)#, print_progress=True)
    max_label = labels.max().item()
    labels = labels.cpu()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(
            labels.numpy() / (max_label if max_label > 0 else 1))
    colors = o3c.Tensor(colors[:, :3], o3c.float32)
    colors[labels < 0] = 0
    pcd.point.colors = colors
    # temppcd = o3d.geometry.PointCloud()
    # temppcd.points = o3d.utility.Vector3dVector([[0, 0, 0], [1, 1, 1], [2, 2, 2]])
    # o3d.visualization.draw_geometries([temppcd])
    # labels = np.array(pcd.cluster_dbscan(eps=1, min_points=10))#, print_progress=True))
    # max_label = labels.max()
    # print(f"point cloud has {max_label + 1} clusters")
    # colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    # colors[labels < 0] = 0
    # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([pcd],
    #                                 zoom=0.455,
    #                                 front=[-0.4999, -0.1659, -0.8499],
    #                                 lookat=[2.1813, 2.0619, 2.0999],
    #                                 up=[0.1204, -0.9852, 0.1215])
    o3d.visualization.draw_geometries([pcd.cpu().to_legacy()]
                                    ,zoom=0.455,
                                    front=[-0.4999, -0.1659, -0.8499],
                                    lookat=[2.1813, 2.0619, 2.0999],
                                    up=[0.1204, -0.9852, 0.1215])
    # print(max_label)

def DBSCAN():
    # DBSCAN.ground = rospy.Publisher('Groundpy', PointCloud2, queue_size=10)
    # DBSCAN.pc = rospy.Publisher('PointCloudpy', PointCloud2, queue_size=10)
    rospy.init_node('Clustering', anonymous=True)
    rospy.Subscriber("rslidar_points", PointCloud2, callback)
    #for i in range(1):callback()
    rospy.spin()

if __name__ == '__main__':
    DBSCAN()
    #callback()
 