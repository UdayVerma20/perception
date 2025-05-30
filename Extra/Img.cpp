#include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <cv_bridge/cv_bridge.h>
// #include <image_transport/image_transport.h>
// #include <opencv2/opencv.hpp>
#include <cmath>
// #include <clustering/CoordinateList.h>
// #include <clustering/Coordinates.h>
#include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/ChannelsFloat32.h>
//publish 
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
// #include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cstdlib>
#include <ctime>

#define Theta 0//0.0175*0.02
//transform eufs
sensor_msgs::PointCloudConstPtr CurrentCameraCluster;
clustering::CoordinateList CameraClusterList;
// CameraClusterList.size = 0;
clustering::CoordinateList FusedClustersList;
std::vector<std::pair<float,float>> LidarList;
// ROS node handle

// ros::Publisher ImgPub;
ros::Subscriber LidarSubscriber;       // ROS subscriber for a topic
ros::Subscriber CameraSubscriber;
// class ImageRectangleNode
// {
// public:
//   ImageRectangleNode()
//   {
//     // Initialize the ImageTransport subscriber
//     // image_transport::ImageTransport it(nh);
//     // image_sub_ = it.subscribe("/camera/rgb/image_color", 1, &ImageRectangleNode::imageCallback, this);
//     std::cout<<"Const1";
//     LidarSubscriber = nh.subscribe("Clusters", 10, &ImageRectangleNode::lidar_callback, this);
//     CameraSubscriber = nh.subscribe("detect_info", 10, &ImageRectangleNode::camera_callback, this);
//     std::cout<<"Const";
//   }

//   // void imageCallback(const sensor_msgs::ImageConstPtr& msg)
//   // {
//   //   try
//   //   {
//   //     // Convert ROS image message to OpenCV image
//   //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

//   //     // Draw a rectangle on the image
//   //     for (int i = 1; i <((*CurrentCameraCluster).channels)[0].values[0]; i++){
//   //     cv::Rect rect(((*CurrentCameraCluster).channels)[i].values[1], ((*CurrentCameraCluster).channels)[i].values[2], ((*CurrentCameraCluster).channels)[i].values[3]-((*CurrentCameraCluster).channels)[i].values[1], ((*CurrentCameraCluster).channels)[i].values[4]-((*CurrentCameraCluster).channels)[i].values[2]);  // (x, y, width, height)
//   //     cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 255, 0), 3);  // Green rectangle, 3px thickness
//   //     }
//   //     for (int i = 0; i <LidarList.size(); i++){
//   //     cv::Rect rect(LidarList[i].first, LidarList[i].second, 0.1, 0.1);  // (x, y, width, height)
//   //     cv::rectangle(cv_ptr->image, rect, cv::Scalar(255, 0, 0), 3);  // Green rectangle, 3px thickness
//   //     }
//   //     // Display the image with the rectangle
//   //     std::cout<<"1";
//   //     cv::imshow("Image with Rectangle", cv_ptr->image);
//   //     cv::waitKey(1);  // Wait for 1 ms to update the display
//   //     std::cout<<"2";

//   //   }
//   //   catch (cv_bridge::Exception& e)
//   //   {
//   //     ROS_ERROR("CV Bridge exception: %s", e.what());
//   //   }
//   // }
//   void camera_callback(const sensor_msgs::PointCloudConstPtr& CameraClusters ) 
//     {
//       std::cout<<"Cam";
//         CurrentCameraCluster = CameraClusters;    
//     }
//     void lidar_callback(const clustering::CoordinateList& LidarClusters ) 
//     { std::cout<<"Lid";
//         // FusedClustersList.size = 0;
//         // FusedClustersList.ConeCoordinates.clear();
//         // iterate list
//         // pcl::PointCloud<pcl::PointXYZ>::Ptr fused_pc (new pcl::PointCloud<pcl::PointXYZ>);
//         // fused_pc->header.frame_id = LidarSource;
//         // tf::TransformListener listener;
//         // tf::StampedTransform transform;
//         // listener.lookupTransform("/zed", "/velodyne",ros::Time(0), transform);
//         // std::cout<<transform<<std::endl;
//         for (int index = 0; index < LidarClusters.size; index++){
//             clustering::Coordinates LidarCluster = LidarClusters.ConeCoordinates[index];
//             LidarCluster.z += 0.05;
//             // LidarCluster.x -= 1.491;
//             // LidarCluster.y -= 0.06;
//             // LidarCluster.z += 0.556;

//             // float Lidar_Rotated_X = LidarCluster.x*cos(Theta) + LidarCluster.z*sin(Theta);
//             // // float Lidar_Rotated_Y = LidarCluster.y;
//             // float Lidar_Rotated_Z = -LidarCluster.x*sin(Theta) + LidarCluster.z*cos(Theta);
//             // LidarCluster.x = Lidar_Rotated_X;
//             // // LidarCluster.y = Lidar_Rotated_Y;
//             // LidarCluster.z = Lidar_Rotated_Z;
//             //LidarCluster.x += 0;
//             //LidarCluster.y += 0;

//             // convert lidar axis to image axis
//             // zc = x, xc = -y, yc = -z
//             std::pair<float,float> LidarTf;
//             LidarTf.first = 525.0 * (- LidarCluster.y)/(LidarCluster.x) + 319.5;
//             LidarTf.second = 525.0 * (- LidarCluster.z)/(LidarCluster.x) + 239.5;
//             LidarList.push_back(LidarTf);  
//              }
//             cv::Mat image = cv::imread("/home/uday/Desktop/download.jpeg");
//             for (int i = 1; i <((*CurrentCameraCluster).channels)[0].values[0]; i++){
//             cv::Rect rect(((*CurrentCameraCluster).channels)[i].values[1], ((*CurrentCameraCluster).channels)[i].values[2], ((*CurrentCameraCluster).channels)[i].values[3]-((*CurrentCameraCluster).channels)[i].values[1], ((*CurrentCameraCluster).channels)[i].values[4]-((*CurrentCameraCluster).channels)[i].values[2]);  // (x, y, width, height)
//             cv::rectangle(image, rect, cv::Scalar(0, 255, 0), 3);  // Green rectangle, 3px thickness
//             }
//             for (int i = 0; i <LidarList.size(); i++){
//             cv::Rect rect(LidarList[i].first, LidarList[i].second, 0.1, 0.1);  // (x, y, width, height)
//             cv::rectangle(image, rect, cv::Scalar(255, 0, 0), 3);  // Green rectangle, 3px thickness
//             }
//             // Display the image with the rectangle
//             std::cout<<"1";
//             cv::imshow("Image with Rectangle", image);
//             cv::waitKey(10);  // Wait for 1 ms to update the display
//             std::cout<<"2";
//             }       

  
//   private:
//     ros::NodeHandle nh;        // ROS node handle
//     // ros::Publisher ImgPub;
//     ros::Subscriber LidarSubscriber;       // ROS subscriber for a topic
//     ros::Subscriber CameraSubscriber;
//     ros::Subscriber YoloSubscriber;
//     // image_transport::Subscriber image_sub_;
// };

cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
int cameracall = 0;
int lidarcall = 0;

void camera_callback(const sensor_msgs::PointCloudConstPtr& CurrentCameraCluster ){
  std::cout << "cam" <<std::endl;
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
  std::cout << "lidar1" <<std::endl;
  if (!cameracall) return;
  if (lidarcall) return;
  std::cout << "lidar2" <<std::endl;
  lidarcall = 1;
  for (int index = 0; index < LidarClusters.size; index++){
    clustering::Coordinates LidarCluster = LidarClusters.ConeCoordinates[index];
    LidarCluster.z += 0.05;
    // LidarCluster.x -= 1.491;
    // LidarCluster.y -= 0.06;
    // LidarCluster.z += 0.556;

    float Lidar_Rotated_X = LidarCluster.x*cos(Theta) + LidarCluster.z*sin(Theta);
    // float Lidar_Rotated_Y = LidarCluster.y;
    float Lidar_Rotated_Z = -LidarCluster.x*sin(Theta) + LidarCluster.z*cos(Theta);
    LidarCluster.x = Lidar_Rotated_X;
    // LidarCluster.y = Lidar_Rotated_Y;
    LidarCluster.z = Lidar_Rotated_Z;
    //LidarCluster.x += 0;
    //LidarCluster.y += 0;

    // convert lidar axis to image axis
    // zc = x, xc = -y, yc = -z
    float Lidar_U = 525.0 * (- LidarCluster.y)/(LidarCluster.x) + 319.5;
    float Lidar_V = 525.0 * (- LidarCluster.z)/(LidarCluster.x) + 239.5; 

    cv::Point point(Lidar_U, Lidar_V);
    cv::Scalar color(254, 254, 254);
    int radius = 3;

    cv::circle(image, point, radius, color, cv::FILLED);
  }
    // Save the image to a file
  // std::string filename = "output_image.png";
  // if (cv::imwrite(filename, image)){
  //     std::cout << "Image saved successfully as " << filename << std::endl;
  // } else {
  //     std::cerr << "Failed to save the image." << std::endl;
  // }

  // Optionally display the image
  cv::imshow("Generated Image", image);
  cv::waitKey(0);
  // ros::Rate(1);
  sleep(10);
  std::exit(EXIT_FAILURE);

}

int main(int argc, char** argv)
{
  // Instantiate the node object
  // ImageRectangleNode node();
  ros::init(argc, argv, "Img");
  ros::NodeHandle nh;
  CameraSubscriber = nh.subscribe("detect_info", 10, camera_callback);
  LidarSubscriber = nh.subscribe("Clusters", 10, lidar_callback);
  
  // Spin to keep the node running
  ros::spin();

  return 0;
}
