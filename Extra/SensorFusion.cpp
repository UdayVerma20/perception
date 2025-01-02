#include <cmath>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/ChannelsFloat32.h>
//publish 
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>
//transform eufs
// #include <tf/transform_listener.h>
using namespace std;
#define LidarSource "rslidar"
#define LidarHeight 0.27 //0.2
// float LidarTransform[] = {};
#define Theta 0//0.0175*0.02
#define margin 0.08
sensor_msgs::PointCloudConstPtr CurrentCameraCluster;
clustering::CoordinateList CameraClusterList;
// CameraClusterList.size = 0;               
clustering::CoordinateList FusedClustersList;
// CameraCluster.header.stamp = ros::Time::now();
// CameraCluster.header.frame_id = LidarSource;
// CameraCluster.size = 0;

// Freenect Camera Matrix
// [525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]
// [235.27027794031173, 0.0, 336.5], [0.0, 235.27027794031173, 188.5], [0.0, 0.0, 1.0]
// Eufs transform
// - Translation: [-1.491, -0.060, 0.556]
// - Rotation: in Quaternion [0.000, -0.009, 0.000, 1.000]
//             in RPY (radian) [0.000, -0.017, 0.000]
//             in RPY (degree) [0.000, -1.000, 0.000]

class SensorFusion
{
public:
    SensorFusion()
    {
        //SensorFusionHandle{};
        FusedCoordinates = SensorFusionHandle.advertise<clustering::CoordinateList>("FusedCoordinates", 10);
        FusedCoordinatesPc = SensorFusionHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("FusedCoordinatesCLoud", 10);
        LidarSubscriber = SensorFusionHandle.subscribe("Clusters", 10, &SensorFusion::lidar_callback, this);
        CameraSubscriber = SensorFusionHandle.subscribe("detect_info", 10, &SensorFusion::camera_callback, this);
        //timer(SensorFusionHandle.createTimer(ros::Duration(0.1), &SensorFusion::main_loop, this));
    }

    void lidar_callback(const clustering::CoordinateList& LidarClusters ) 
    {
        FusedClustersList.size = 0;
        FusedClustersList.ConeCoordinates.clear();
        vector<clustering::Coordinates> sortedClusters;
        for(int i=0; i<LidarClusters.size; i++){
           sortedClusters.push_back(LidarClusters.ConeCoordinates[i]);
        }
        for(int i=0; i<sortedClusters.size(); i++){
            for(int j=0; j<sortedClusters.size()-i-1; j++){
                float distnextj = pow(pow(sortedClusters[j+1].x, 2) + pow(sortedClusters[j+1].y, 2), 0.5);
                float distj = pow(pow(sortedClusters[j].x, 2) + pow(sortedClusters[j].y, 2), 0.5);
                if(distj>distnextj){
                    swap(sortedClusters[j+1], sortedClusters[j]);
                }
            }
        }
        // for (auto i : sortedClusters){
        //     cout<<pow(pow(i.x, 2) + pow(i.y, 2), 0.5)<<" ";
        // }
        cout<<endl;
        // iterate list
        pcl::PointCloud<pcl::PointXYZ>::Ptr fused_pc (new pcl::PointCloud<pcl::PointXYZ>);
        fused_pc->header.frame_id = LidarSource;
        // tf::TransformListener listener;
        // tf::StampedTransform transform;
        // listener.lookupTransform("/zed", "/velodyne",ros::Time(0), transform);
        // cout<<transform<<endl;
        vector<bool> CameraBoxTook(((*CurrentCameraCluster).channels)[0].values[0],1);
        for (int index = 0; index < sortedClusters.size(); index++){
            clustering::Coordinates LidarCluster = sortedClusters[index];
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
            // cout<<Lidar_U << " "<<Lidar_V<<endl;
            // for (int nestedindex = 0; nestedindex < CameraClusterList.size; nestedindex++){
            //     clustering::Coordinates CameraCluster = CameraClusterList.ConeCoordinates[nestedindex];
            //     if (abs(Lidar_U - CameraCluster.x) < (float)300/log(LidarCluster.x)){ //100){
            //         if (abs(Lidar_V - CameraCluster.x) < (float)300/log(LidarCluster.x)){ //100){
            //             clustering::Coordinates CurrentCluster;
            //             CurrentCluster.x = - LidarCluster.y; //rotate x and y
            //             CurrentCluster.y = LidarCluster.x;
            //             CurrentCluster.z = LidarCluster.z -0.05;
            //             CurrentCluster.colour = CameraCluster.colour;
            //             cout<<CameraCluster.colour<<endl;
            //             fused_pc->push_back(pcl::PointXYZ(LidarCluster.x, LidarCluster.y, LidarCluster.z-0.05));
            //             FusedClustersList.ConeCoordinates.push_back(CurrentCluster);
            //             (FusedClustersList.size)++;
            //             break;
            //         }
            //     }
            // }
            for (int nested_index = 1; nested_index <= ((*CurrentCameraCluster).channels)[0].values[0] ;nested_index++){
                auto Camera_Cone = ((*CurrentCameraCluster).channels)[nested_index];
                if ((Camera_Cone.values[0] > 0.3)
                 && (CameraBoxTook[nested_index])
                 && ((Lidar_U>= (1+margin)*Camera_Cone.values[1] - margin*Camera_Cone.values[3]) && (Lidar_U<= (1+margin)*Camera_Cone.values[3] - margin*Camera_Cone.values[1]))
                 && ((Lidar_V>= (1+margin)*Camera_Cone.values[2] - margin*Camera_Cone.values[4]) && (Lidar_V<= (1+margin)*Camera_Cone.values[4] - margin*Camera_Cone.values[2]))
                 ){
                    clustering::Coordinates CurrentCluster;
                    CurrentCluster.x = - LidarCluster.y;
                    CurrentCluster.y = LidarCluster.x;
                    CurrentCluster.z = LidarCluster.z -0.05;
                    CurrentCluster.colour = ((Camera_Cone.name).compare("blue_cone") == 0) ? 1:0;
                    fused_pc->push_back(pcl::PointXYZ(sortedClusters[index].x, sortedClusters[index].y, sortedClusters[index].z));
                    FusedClustersList.ConeCoordinates.push_back(CurrentCluster);
                    (FusedClustersList.size)++;
                    // CurrentCameraCluster.erase(next(CurrentCameraCluster.begin(),nested_index));
                    CameraBoxTook[nested_index]=0;
                    break;
                }   
            }
        }
        FusedCoordinatesPc.publish(*fused_pc);
        FusedCoordinates.publish(FusedClustersList);
        cout<<"Printing fused coordinates"<<endl;
        // cout<<FusedClustersList.size<<" Clusters"<<endl;
    }

    void camera_callback(const sensor_msgs::PointCloudConstPtr& CameraClusters ) 
    {
        // CameraClusterList.header.stamp = ros::Time::now();
        // // CameraCluster.header.frame_id = LidarSource;
        // CameraClusterList.size = 0;
        // CameraClusterList.ConeCoordinates.clear();
        // for (int index = 1; index <= ((*CameraClusters).channels)[0].values[0] ;index++){
        //     auto Cone = ((*CameraClusters).channels)[index];
        //   if (Cone.values[0] > 0.3){
        //     // cout <<"1";
        //     clustering::Coordinates CurrentCluster;
        //     CurrentCluster.x = (Cone.values[1] + Cone.values[3])/2;
        //     CurrentCluster.y = Cone.values[2];
        //     CurrentCluster.z = 0;
        //     CurrentCluster.colour = ((Cone.name).compare("blue_cone") == 0) ? 1:0 ;
        //     // cout<<CurrentCluster.colour<<"a"<<endl;
        //     // cout<<CurrentCluster.colour;
        //     CameraClusterList.ConeCoordinates.push_back(CurrentCluster);
        //     CameraClusterList.size ++;
        //   }   
        // }
        CurrentCameraCluster = CameraClusters;    
    }


private:
    ros::NodeHandle SensorFusionHandle;        // ROS node handle
    ros::Publisher FusedCoordinates;
    ros::Publisher FusedCoordinatesPc;
    ros::Subscriber LidarSubscriber;       // ROS subscriber for a topic
    ros::Subscriber CameraSubscriber;
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SensorFusion");
    SensorFusion node;
    ros::spin();
    return 0;
}

