#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>
#include <ctime>

#define Theta 0.0174533*0
sensor_msgs::PointCloudConstPtr CurrentCameraCluster;
clustering::CoordinateList CameraClusterList;
clustering::CoordinateList FusedClustersList;
std::vector<std::pair<float,float>> LidarList;

ros::Subscriber LidarSubscriber;
ros::Subscriber CameraSubscriber;
cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
int cameracall = 0;
int lidarcall = 0;

void camera_callback(const sensor_msgs::PointCloudConstPtr& CurrentCameraCluster ){
  cameracall = 1;
  for (int nested_index = 1; nested_index <= ((*CurrentCameraCluster).channels)[0].values[0] ;nested_index++){
    auto Camera_Cone = ((*CurrentCameraCluster).channels)[nested_index];
    cv::Point pt1(Camera_Cone.values[1], Camera_Cone.values[2]);
    cv::Point pt2(Camera_Cone.values[3], Camera_Cone.values[4]);
    cv::Scalar color((Camera_Cone.name=="yellow_cone"?0:254), (Camera_Cone.name=="yellow_cone"?254:0), 0);
    int thickness = 3;
    cv::rectangle(image, pt1, pt2, color, thickness);
  }
}

void lidar_callback(const clustering::CoordinateList& LidarClusters ){ 
  if (!cameracall) return;
  if (lidarcall) return;
  lidarcall = 1;
  for (int index = 0; index < LidarClusters.size; index++){
    clustering::Coordinates LidarCluster = LidarClusters.ConeCoordinates[index];
    LidarCluster.x += 1.5;
    LidarCluster.y -= 0.06;
    LidarCluster.z -= 0.59;

    float Lidar_Rotated_Y = LidarCluster.y*cos(Theta) - LidarCluster.z*sin(Theta);
    float Lidar_Rotated_Z = LidarCluster.y*sin(Theta) + LidarCluster.z*cos(Theta);
    LidarCluster.y = Lidar_Rotated_Y;
    LidarCluster.z = Lidar_Rotated_Z;
    float Lidar_U = 525.0 * (- LidarCluster.y)/(LidarCluster.x) + 360.5;
    float Lidar_V = 525.0 * (- LidarCluster.z)/(LidarCluster.x) + 295.5; 
    cv::Point point(Lidar_U, Lidar_V);
    cv::Scalar color(254, 254, 254);
    int radius = 5;
    cv::circle(image, point, radius, color, cv::FILLED);
  }
  cv::imshow("Generated Image", image);
  cv::waitKey(0);
  sleep(10);
  std::exit(EXIT_FAILURE);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Img");
  ros::NodeHandle nh;
  CameraSubscriber = nh.subscribe("detect_info", 10, camera_callback);
  LidarSubscriber = nh.subscribe("Clusters_temp", 10, lidar_callback);
  ros::spin();
  return 0;
}
