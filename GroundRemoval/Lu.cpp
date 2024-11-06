#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/filter.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <cstdlib>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include<bits/stdc++.h>
#define heightlidar 0.20 //0.188
#define ringlength 0.4 //1.5
#define sectorangle M_PI/12 //sector angle has to be a multiple of pi/6
#define startatring 0
#define planespreadtillring 35
#define distancethreshold 0.06 //0.13
#define lidarframe "rslidar" //"rslidar"

ros::Publisher pub;
ros::Publisher pub_;
ros::Publisher pub_cloud;
ros::Publisher marker_pub;
std::unordered_map<int, pcl::PointXYZI> bin_min_points;
std::unordered_map<int, std::vector<pcl::PointXYZI>> bin_all_points;
std::unordered_map<int, std::vector<float>> plane;
std::vector<std::vector<pcl::PointXYZI>> visualisationlist;

int markerid = 0;





void propogate (pcl::PointXYZI lastpoint, int segment1, int segment2, int ringcount){
  if(ringcount>=planespreadtillring) return;
  if(bin_min_points.find(segment1)==bin_min_points.end() || bin_min_points.find(segment2)==bin_min_points.end()){
    if(bin_min_points.find(segment1)!=bin_min_points.end()) plane[segment1] = {0.f, 0.f, 1.f, -bin_min_points[segment1].z};
    else if(bin_min_points.find(segment2)!=bin_min_points.end()) plane[segment1] = {0.f, 0.f, 1.f, -bin_min_points[segment2].z};

    propogate(lastpoint, segment1+2*M_PI/(3*sectorangle), segment2+2*M_PI/(3*sectorangle), ringcount+1);
    return;
  }
  float x1 = lastpoint.x;
  float y1 = lastpoint.y;
  float z1 = lastpoint.z;

  float x2 = bin_min_points[segment1].x;
  float y2 = bin_min_points[segment1].y;
  float z2 = bin_min_points[segment1].z;

  float x3 = bin_min_points[segment2].x;
  float y3 = bin_min_points[segment2].y;
  float z3 = bin_min_points[segment2].z;
 
  visualisationlist.push_back({lastpoint, bin_min_points[segment1], bin_min_points[segment2], lastpoint});

  float a = y2*z3 + y3*z1 + y1*z2 - (y1*z3 + y2*z1 + y3*z2);
  float b = x3*z2 + x1*z3 + x2*z1 - (x3*z1 + x1*z2 + x2*z3);
  float c = x2*y3 + x1*y2 + x3*y1 - (x2*y1 + x1*y3 + x3*y2);
  float d = -x1*a - y1*b - z1*c;
  plane[segment1] = {a, b, c, d};
  lastpoint.x = (x2+x3)/2; lastpoint.y = (y2+y3)/2; lastpoint.z = (z2+z3)/2;
  propogate(lastpoint, segment1+2*M_PI/(3*sectorangle), segment2+2*M_PI/(3*sectorangle), ringcount+1);
}

void visualise(){
  visualization_msgs::MarkerArray visualisationarray;
  //making rings
  for(int i=0; i<planespreadtillring; i++){
    visualization_msgs::Marker ringmarker;
    ringmarker.header.frame_id = lidarframe;
    ringmarker.header.stamp = ros::Time::now();
    ringmarker.ns = "basic_shapes";
    ringmarker.id = markerid++;
    ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
    ringmarker.action = visualization_msgs::Marker::ADD;
    ringmarker.lifetime = ros::Duration(0.1);

    ringmarker.pose.orientation.w=1.0;
    ringmarker.scale.x = 0.005;
    ringmarker.color.r = 1.0;
    ringmarker.color.a = 1.0;
    for(int j=0; j<13; j++){
      geometry_msgs::Point ringpoint;
      ringpoint.x = i*ringlength*sin(j*M_PI/18 + M_PI/6);
      ringpoint.y = -i*ringlength*cos(j*M_PI/18 + M_PI/6);
      ringpoint.z = -heightlidar;
      ringmarker.points.push_back(ringpoint);
    }
    visualisationarray.markers.push_back(ringmarker);
  }

  //making sectors
  for(int i=0; i<=2*M_PI/(3*sectorangle); i++){
    visualization_msgs::Marker ringmarker;
    ringmarker.header.frame_id = lidarframe;
    ringmarker.header.stamp = ros::Time::now();
    ringmarker.ns = "basic_shapes";
    ringmarker.id = markerid++;
    ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
    ringmarker.action = visualization_msgs::Marker::ADD;
    ringmarker.lifetime = ros::Duration(0.1);

    ringmarker.pose.orientation.w=1.0;
    ringmarker.scale.x = 0.005;
    ringmarker.color.r = 1.0;
    ringmarker.color.a = 1.0;
   
    geometry_msgs::Point sectorpoint1;
    sectorpoint1.x = 0;sectorpoint1.y=0;sectorpoint1.z=-heightlidar;
    ringmarker.points.push_back(sectorpoint1);
    geometry_msgs::Point sectorpoint2;
    sectorpoint2.x = planespreadtillring*ringlength*sin(i*sectorangle + M_PI/6);sectorpoint2.y=-planespreadtillring*ringlength*cos(i*sectorangle + M_PI/6);sectorpoint2.z=-heightlidar;
    ringmarker.points.push_back(sectorpoint2);
    visualisationarray.markers.push_back(ringmarker);
  }

  //making planes
  for(auto total:visualisationlist){
    visualization_msgs::Marker ringmarker;
    ringmarker.header.frame_id = lidarframe;
    ringmarker.header.stamp = ros::Time::now();
    ringmarker.ns = "basic_shapes";
    ringmarker.id = markerid++;
    ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
    ringmarker.action = visualization_msgs::Marker::ADD;
    ringmarker.lifetime = ros::Duration(0.1);

    ringmarker.pose.orientation.w=1.0;
    ringmarker.scale.x = 0.005;
    ringmarker.color.g = 1.0;
    ringmarker.color.a = 1.0;

    for(auto i:total){
      geometry_msgs::Point minpoint;
      minpoint.x = i.x;minpoint.y = i.y;minpoint.z = i.z;
      ringmarker.points.push_back(minpoint);
    }
    visualisationarray.markers.push_back(ringmarker);
  }
  ROS_INFO("Visualising to /visualization_marker");
  marker_pub.publish(visualisationarray);
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{ 
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*input, *cloud);

    pcl::Indices indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloudtomanipulate, indices);
    // cloudtomanipulate->header.frame_id = "rslidar";

    bin_min_points.clear();
    bin_all_points.clear();
    plane.clear();
    visualisationlist.clear();

    //segregating points into bins and placing them in bin_min_points and bin_all_points
    for(int index=0; index<cloudtomanipulate->points.size(); index++){
    int distance = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(distance>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;

    //trimming view area
    if((cloudtomanipulate->points)[index].y >3.5 || (cloudtomanipulate->points)[index].y<-2.5) continue;

    //this key starts from 0 from right direction and denotes the bin number. It increases as we move towards left or farther away from lidar
    int key = int((angle)/(sectorangle)) + int((2*M_PI*distance)/(3*sectorangle));

    //adding to bin_all_points
    bin_all_points[key].push_back((cloudtomanipulate->points)[index]);

    //adding to bin_min_points
    if(bin_min_points.find(key)!=bin_min_points.end()){
        if((cloudtomanipulate->points)[index].z<bin_min_points[key].z){
        bin_min_points[key] = (cloudtomanipulate->points)[index];
        }
    }else{
        bin_min_points[key] = (cloudtomanipulate->points)[index];
    }
    }

    //propogating the planes in all the bins
    pcl::PointXYZI lidarpoint;
    lidarpoint.x = 0.f;
    lidarpoint.y = 0.f;
    lidarpoint.z = -heightlidar;
    for(int i=int(2*M_PI*startatring/(3*sectorangle)); i<int(2*M_PI*(startatring+1)/(3*sectorangle)); i+=2){
    propogate(lidarpoint, i, i+1, startatring);
    }

    //visualising
    visualise();

    //removing points in the viscinity of the planes
    std::vector<float> coeffs(4, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>);
    nonground->header.frame_id = lidarframe;
    for(auto i:bin_all_points){
    // if(i.first>planespreadtillring*8){
    //   continue;
    // }
    if(i.first%2==0){
        coeffs = plane[i.first];
    }else{
        coeffs = plane[i.first-1];
    }
    float denominator = pow(pow(coeffs[0], 2) + pow(coeffs[1], 2) + pow(coeffs[2], 2), 0.5);
    for(auto j:i.second){
        if(abs((coeffs[0]*j.x + coeffs[1]*j.y + coeffs[2]*j.z + coeffs[3])/denominator)>=distancethreshold) nonground->push_back(j);
    }
    }

    // Publish the data.
    ROS_INFO("Publishing to nogroundcloud");
    // ros::Rate rate(4);
    pub.publish (*nonground);
    // rate.sleep();
}
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "luphy");
  ros::NodeHandle nh;
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ConeCloud", 1);
  // pub_ = nh.advertise<perception::CoordinateList>("Clusters", 1);
  // pub_cloud= nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("ClustersCloud", 1);
  marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
  ros::Subscriber sub = nh.subscribe ("VoxelCloud", 1, cloud_cb);
  // rate.sleep
  ros::spin ();
}