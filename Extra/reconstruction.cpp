
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

#define lidarframe "rslidar"
#define heightcuboid 0.35
#define lengthcuboid 0.1 //front and back of cone
#define widthcuboid 0.2 //side to side of cone

perception::CoordinateList clusters;
ros::Publisher reconground;
ros::Publisher reconcluster;
ros::Publisher reconclusterpc;

void cluster_cb (perception::CoordinateList inputclusters)
{
  clusters = inputclusters;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructedcloud(new pcl::PointCloud<pcl::PointXYZI>);
  reconstructedcloud->header.frame_id = lidarframe;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloudtomanipulate);
  for(int index=0; index<cloudtomanipulate->points.size(); index++){
    for(int cluster_index = 0; cluster_index < clusters.size; cluster_index++){
        if((cloudtomanipulate->points)[index].x >= clusters.ConeCoordinates[cluster_index].x - lengthcuboid && (cloudtomanipulate->points)[index].x <= clusters.ConeCoordinates[cluster_index].x + lengthcuboid
        && (cloudtomanipulate->points)[index].y >= clusters.ConeCoordinates[cluster_index].y - widthcuboid && (cloudtomanipulate->points)[index].y <= clusters.ConeCoordinates[cluster_index].y + widthcuboid
        && (cloudtomanipulate->points)[index].z >= clusters.ConeCoordinates[cluster_index].z - heightcuboid && (cloudtomanipulate->points)[index].z <= clusters.ConeCoordinates[cluster_index].z + heightcuboid        
        ) {
            reconstructedcloud->push_back((cloudtomanipulate->points)[index]);
            clusters.ConeCoordinates[cluster_index].size++;
            if (clusters.ConeCoordinates[cluster_index].left[1] > (cloudtomanipulate->points)[index].y){
                clusters.ConeCoordinates[cluster_index].left = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            }
            if (clusters.ConeCoordinates[cluster_index].right[1] < (cloudtomanipulate->points)[index].y){
                clusters.ConeCoordinates[cluster_index].right = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            }
            if (clusters.ConeCoordinates[cluster_index].top[2] < (cloudtomanipulate->points)[index].z){
                clusters.ConeCoordinates[cluster_index].top = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            }
            if (clusters.ConeCoordinates[cluster_index].bottom[2] > (cloudtomanipulate->points)[index].y){
                clusters.ConeCoordinates[cluster_index].left = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            }
        }
    }
  }
  ROS_INFO("Publishing to ReconstructedGround");
  reconground.publish (*reconstructedcloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructedcluster(new pcl::PointCloud<pcl::PointXYZI>);
  reconstructedcluster->header.frame_id = lidarframe;
  reconstructedcluster->push_back(pcl::PointXYZI(0.f));
  for (int i = 0; i < clusters.size; i++){
    perception::Coordinates Iter_Cluster = clusters.ConeCoordinates[i];
    int dist_sq = pow(Iter_Cluster.x, 2.0) + pow(Iter_Cluster.y, 2.0) + pow(Iter_Cluster.z, 2.0);
    int expected_points = (5000)/(dist_sq + 1);
    if ( //checks
    0
    // // (distclusfromground >=0.1 && distclusfromground < 0.22)
    // // && (Iter_Cluster.z + LidarHeight < MaxHeight)
    // // && (Iter_Cluster.z + LidarHeight > MinHeight)
    // // && (maxzfromground >= 0.21 )
    // // && (minzfromground <= 0.1 )
    // && (Iter_Cluster.size < expected_points)
    // && (Iter_Cluster.size > 0.1 * expected_points)
    // && ((Iter_Cluster.right[1] - Iter_Cluster.left[1])*(Iter_Cluster.right[1] - Iter_Cluster.left[1]) + (Iter_Cluster.right[0] - Iter_Cluster.left[0])*(Iter_Cluster.right[0] - Iter_Cluster.left[0]) < MaxWidth*MaxWidth)
    // && (Iter_Cluster.size > MinPoints)
    // && (Iter_Cluster.x*Iter_Cluster.x + Iter_Cluster.y*Iter_Cluster.y < 30)
    // && (curr_colour == 0)
    ){
      pcl::PointXYZI point;
      point.x = Iter_Cluster.x;
      point.y = Iter_Cluster.y;
      point.z = Iter_Cluster.z;
      reconstructedcluster->push_back(point);
    }
    else{
      clusters.ConeCoordinates.erase(clusters.ConeCoordinates.begin() + i);
    }
  }
  
  ROS_INFO("Publishing to ReconstructedCluster and ReconstructedPc");
  reconcluster.publish(clusters);
  reconclusterpc.publish(reconstructedcluster);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstruct");
  ros::NodeHandle nh;

  reconground = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ReconstructedGround", 1);
  reconclusterpc = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ReconstructedClusterPc", 1);
  reconcluster = nh.advertise<perception::CoordinateList> ("ReconstructedCluster", 1);

  ros::Subscriber subcluster = nh.subscribe ("Clusters", 1, cluster_cb);
  ros::Subscriber subcloud = nh.subscribe ("rslidar_points", 1, cloud_cb);
  
  ros::spin();
  return 0;
}
