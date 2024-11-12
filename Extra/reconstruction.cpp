
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/filter.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
#include <ground_removal/arr.h>
#include <ground_removal/arrofarr.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

#define lidarframe "rslidar"
#define heightcuboid 0.35
#define lengthcuboid 0.1 //front and back of cone
#define widthcuboid 0.2 //side to side of cone
#define heightlidar 0.25 
#define lidarframe "rslidar" //"rslidar"
#define sectorangle M_PI/48
#define planespreadbeforering 70
#define ringlength 0.2
#define xthresh 3

clustering::CoordinateList clusters;
unordered_map<int, vector<float>> ground;
ros::Publisher reconground;
ros::Publisher reconcluster;
ros::Publisher reconclusterpc;

void cluster_cb (clustering::CoordinateList inputclusters)
{
  clusters = inputclusters;
}

void plane_cb(ground_removal::arrofarr inputplanes)
{
  ground.clear();
  for(auto i:inputplanes.data){
      ground[int(round(i.data[0]))] = {i.data[1], i.data[2], i.data[3], i.data[4]};
  }}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructedcloud(new pcl::PointCloud<pcl::PointXYZI>);
  reconstructedcloud->header.frame_id = lidarframe;
  reconstructedcloud->push_back(pcl::PointXYZI(0.f));
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloudtomanipulate);
  for(int index=0; index<cloudtomanipulate->points.size(); index++){
    for(int cluster_index = 0; cluster_index < clusters.size; cluster_index++){
        if(((cloudtomanipulate->points)[index].x >= clusters.ConeCoordinates[cluster_index].x - lengthcuboid && (cloudtomanipulate->points)[index].x <= clusters.ConeCoordinates[cluster_index].x + lengthcuboid)
        && ((cloudtomanipulate->points)[index].y >= clusters.ConeCoordinates[cluster_index].y - widthcuboid && (cloudtomanipulate->points)[index].y <= clusters.ConeCoordinates[cluster_index].y + widthcuboid)
        && ((cloudtomanipulate->points)[index].z >= clusters.ConeCoordinates[cluster_index].z - heightcuboid && (cloudtomanipulate->points)[index].z <= clusters.ConeCoordinates[cluster_index].z + heightcuboid)        
        ){
            // Not updating avg
            reconstructedcloud->push_back((cloudtomanipulate->points)[index]);
            clusters.ConeCoordinates[cluster_index].reconsize++;
            // if (clusters.ConeCoordinates[cluster_index].front[0] > (cloudtomanipulate->points)[index].x){
            //     clusters.ConeCoordinates[cluster_index].front = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
            // if (clusters.ConeCoordinates[cluster_index].rear[0] < (cloudtomanipulate->points)[index].x){
            //     clusters.ConeCoordinates[cluster_index].rear = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
            // if (clusters.ConeCoordinates[cluster_index].left[1] > (cloudtomanipulate->points)[index].y){
            //     clusters.ConeCoordinates[cluster_index].left = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
            // if (clusters.ConeCoordinates[cluster_index].right[1] < (cloudtomanipulate->points)[index].y){
            //     clusters.ConeCoordinates[cluster_index].right = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
            // if (clusters.ConeCoordinates[cluster_index].top[2] < (cloudtomanipulate->points)[index].z){
            //     clusters.ConeCoordinates[cluster_index].top = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
            // if (clusters.ConeCoordinates[cluster_index].bottom[2] > (cloudtomanipulate->points)[index].z){
            //     clusters.ConeCoordinates[cluster_index].bottom = {(cloudtomanipulate->points)[index].x,(cloudtomanipulate->points)[index].y,(cloudtomanipulate->points)[index].z};
            // }
        }
    }
  }
  ROS_INFO("Publishing to ReconstructedGround");
  reconground.publish (*reconstructedcloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr reconstructedcluster(new pcl::PointCloud<pcl::PointXYZI>);
  reconstructedcluster->header.frame_id = lidarframe;
  reconstructedcluster->push_back(pcl::PointXYZI(0.f));
  clustering::CoordinateList recon_clusters;
  recon_clusters.size = 0;
  float denominator = 1;
  for (int i = 0; i < clusters.size; i++){
    clustering::Coordinates Iter_Cluster = clusters.ConeCoordinates[i];
    // int dist_sq = pow(Iter_Cluster.x, 2.0) + pow(Iter_Cluster.y, 2.0) + pow(Iter_Cluster.z, 2.0);
    // int expected_points = (2000)/(dist_sq + 1); //remove dist =0
    // if (!(Iter_Cluster.Avg.x == 0 && Iter_Cluster.Avg.y ==0)){
    //     float ratio = Iter_Cluster.Avg.y/abs(Iter_Cluster.Avg.x);
    //     if ((ratio > tan(10) && ratio < tan(15)) || (ratio > tan(30) && ratio < tan(40))){
    //         expected_points /= 2;
    //     }
    // }
    
    //checking cone distance from lu planes
    // int ringclus = pow(pow(Iter_Cluster.x, 2)+pow(Iter_Cluster.y, 2), 0.5)/ringlength;
    // float angleclus = atan2(Iter_Cluster.x, -Iter_Cluster.y) - M_PI/6;
    // int key = int((angleclus)/(sectorangle)) + int((2*M_PI*ringclus)/(3*sectorangle));
    
    // denominator = pow(pow(ground[key][0], 2) + pow(ground[key][1], 2) + pow(ground[key][2], 2), 0.5);
    // float distance = abs(Iter_Cluster.x*ground[key][0] + Iter_Cluster.y*ground[key][1] + Iter_Cluster.z*ground[key][2] + ground[key][3])/denominator;
    // float minzfromground = abs(Iter_Cluster.bottom[0]*ground[key][0] + Iter_Cluster.bottom[1]*ground[key][1] + Iter_Cluster.bottom[2]*ground[key][2] + ground[key][3])/denominator;
    // float maxzfromground = abs(Iter_Cluster.top[0]*ground[key][0] + Iter_Cluster.top[1]*ground[key][1] + Iter_Cluster.top[2]*ground[key][2] + ground[key][3])/denominator;
    
    if ( //checks
    1
    && ((abs(Iter_Cluster.size - Iter_Cluster.reconsize)/Iter_Cluster.size) < 1)
    // && ( Iter_Cluster.reconsize>=expected_points*0.2 )
    // && (Iter_Cluster.reconsize > MinPoints)
    // && ( Iter_Cluster.reconsize<=expected_points )
    // && ( Iter_Cluster.top[2]-Iter_Cluster.bottom[2] > 0.1 )
    // && ( Iter_Cluster.top[2]-Iter_Cluster.bottom[2] < 0.5 )
    // && ( abs(Iter_Cluster.left[1]-Iter_Cluster.right[1]) > 0.02 )
    // && ( abs(Iter_Cluster.left[1]-Iter_Cluster.right[1]) < 0.2 )
    // // && (Iter_Cluster.z + LidarHeight < MaxHeight)
    // // && (Iter_Cluster.z + LidarHeight > MinHeight)
    // // && (maxzfromground >= 0.21 )
    // // && (minzfromground <= 0.1 )
    // && ((Iter_Cluster.right[1] - Iter_Cluster.left[1])*(Iter_Cluster.right[1] - Iter_Cluster.left[1]) + (Iter_Cluster.right[0] - Iter_Cluster.left[0])*(Iter_Cluster.right[0] - Iter_Cluster.left[0]) < MaxWidth*MaxWidth)
    // && (Iter_Cluster.x*Iter_Cluster.x + Iter_Cluster.y*Iter_Cluster.y < 30)
    
    ){
      pcl::PointXYZI point;
      point.x = Iter_Cluster.x;
      point.y = Iter_Cluster.y;
      point.z = Iter_Cluster.z;
      reconstructedcluster->push_back(point);
      recon_clusters.ConeCoordinates.push_back(Iter_Cluster);
      recon_clusters.size++;
      // cout<<"no"<<endl;
    }
    // else{
    //   cout<<"hi"<<endl;
    //   clusters.ConeCoordinates.erase(clusters.ConeCoordinates.begin() + i);
    // }
  }
  
  ROS_INFO("Publishing to ReconstructedCluster and ReconstructedPc");
  reconcluster.publish(recon_clusters);
  reconclusterpc.publish(reconstructedcluster);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reconstruct");
  ros::NodeHandle nh;

  reconground = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ReconstructedGround", 1);
  reconclusterpc = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ReconstructedClusterPc", 1);
  reconcluster = nh.advertise<clustering::CoordinateList> ("ReconstructedCluster", 1);

  ros::Subscriber subcluster = nh.subscribe ("Clusters", 1, cluster_cb);
  ros::Subscriber subcloud = nh.subscribe ("rslidar_points", 1, cloud_cb);
  ros::Subscriber subplanes = nh.subscribe ("ground", 1, plane_cb);
  ros::spin();
  return 0;
}
