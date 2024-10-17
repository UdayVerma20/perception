
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
  ROS_INFO("Publishing to ReconstructedGround & ReconstructedCluster");
  reconground.publish (*reconstructedcloud);
  reconcluster.publish(clusters);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstruct");
  ros::NodeHandle nh;

  reconground = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ReconstructedGround", 1);
  reconcluster = nh.advertise<perception::CoordinateList> ("ReconstructedCluster", 1);

  ros::Subscriber subcluster = nh.subscribe ("Clusters", 1, cluster_cb);
  ros::Subscriber subcloud = nh.subscribe ("rslidar_points", 1, cloud_cb);
  
  ros::spin();
  return 0;
}
