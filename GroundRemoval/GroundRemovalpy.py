#!/usr/bin/env python3
import rospy
import std_msgs.msg
from sensor_msgs.msg import PointCloud2
import open3d as o3d
import open3d.core as o3c
from open3d_ros_helper import open3d_ros_helper as orh
import numpy as np

def callback(data):
    o3dpc = orh.rospc_to_o3dpc(data)
    pcd = o3d.t.geometry.PointCloud(o3c.Tensor(np.asarray(o3dpc.points, dtype = np.float32), o3c.float32, o3c.Device("cuda:0")))
    #o3dpc = o3d.data.PLYPointCloud()
    #pcd = o3d.t.io.read_point_cloud('/home/uday/open3d_data/download/PLYPointCloud/fragment.ply')
    #pcd = pcd.cuda()
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.1,
                                            ransac_n=100,
                                            num_iterations=1000)
    [a, b, c, d] = plane_model.cpu().numpy().tolist()
    #inliers = inliers[:10]
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    inlier_cloud = orh.o3dpc_to_rospc(pcd.select_by_index(inliers).to_legacy())
    inlier_cloud.header.frame_id = "rslidar"
    outlier_cloud = orh.o3dpc_to_rospc(pcd.select_by_index(inliers, invert=True).to_legacy())
    outlier_cloud.header.frame_id = "rslidar"
    ground_removal.ground.publish(inlier_cloud)#orh.o3dpc_to_rospc(inlier_cloud))
    ground_removal.pc.publish(outlier_cloud)#orh.o3dpc_to_rospc(outlier_cloud))
    #o3d.visualization.draw_geometries([outlier_cloud])
    print("Sent \n")

def ground_removal():
    ground_removal.ground = rospy.Publisher('Groundpy', PointCloud2, queue_size=10)
    ground_removal.pc = rospy.Publisher('PointCloudpy', PointCloud2, queue_size=10)
    rospy.init_node('GroundRemovalpy', anonymous=True)
    rospy.Subscriber("VoxelCloud", PointCloud2, callback)
    #for i in range(1):callback()
    rospy.spin()

if __name__ == '__main__':
    ground_removal()
    #callback()
 