#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
// #include <sensor_msgs/ChannelsFloat32.h>
//publish 
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>

#define LidarSource "rslidar"
#define LidarHeight 0.18 //0.2
// float LidarTransform[] = {};

perception::CoordinateList CameraClusterList;
// CameraClusterList.size = 0;
perception::CoordinateList FusedClustersList;
// CameraCluster.header.stamp = ros::Time::now();
// CameraCluster.header.frame_id = LidarSource;
// CameraCluster.size = 0;

// Freenect Camera Matrix
// [525.0, 0.0, 319.5], [0.0, 525.0, 239.5], [0.0, 0.0, 1.0]

class SensorFusion
{
public:
    SensorFusion()
    {
        //SensorFusionHandle{};
        FusedCoordinates = SensorFusionHandle.advertise<perception::CoordinateList>("FusedCoordinates", 10);
        FusedCoordinatesPc = SensorFusionHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("FusedCoordinatesCLoud", 10);
        LidarSubscriber = SensorFusionHandle.subscribe("Clusters", 10, &SensorFusion::lidar_callback, this);
        CameraSubscriber = SensorFusionHandle.subscribe("detect_info", 10, &SensorFusion::camera_callback, this);
        //timer(SensorFusionHandle.createTimer(ros::Duration(0.1), &SensorFusion::main_loop, this));
    }

    void lidar_callback(const perception::CoordinateList& LidarClusters ) 
    {
        FusedClustersList.size = 0;
        FusedClustersList.ConeCoordinates.clear();
        // iterate list
        pcl::PointCloud<pcl::PointXYZ>::Ptr fused_pc (new pcl::PointCloud<pcl::PointXYZ>);
        fused_pc->header.frame_id = LidarSource;
        for (int index = 0; index < LidarClusters.size; index++){
            perception::Coordinates LidarCluster = LidarClusters.ConeCoordinates[index];
            LidarCluster.z += 0.05;
            //LidarCluster.x += 0;
            //LidarCluster.y += 0;
            
            // convert lidar axis to image axis
            // zc = x, xc = -y, yc = -z
            for (int nestedindex = 0; nestedindex < CameraClusterList.size; nestedindex++){
                perception::Coordinates CameraCluster = CameraClusterList.ConeCoordinates[nestedindex];
                if (abs(525 * (- LidarCluster.y)/(LidarCluster.x) + 319.5 - CameraCluster.x) < 100){
                    if ((525 * (- LidarCluster.z)/(LidarCluster.x) + 239.5 - CameraCluster.x) < 100){
                        perception::Coordinates CurrentCluster;
                        CurrentCluster.x = - LidarCluster.y;
                        CurrentCluster.y = LidarCluster.x;
                        CurrentCluster.z = LidarCluster.z -0.05;
                        CurrentCluster.colour = CameraCluster.colour;
                        fused_pc->push_back(pcl::PointXYZ(LidarCluster.x, LidarCluster.y, LidarCluster.z-0.05));
                        FusedClustersList.ConeCoordinates.push_back(CurrentCluster);
                        (FusedClustersList.size)++;
                        break;
                    }
                }
            }
        }
        FusedCoordinatesPc.publish(*fused_pc);
        FusedCoordinates.publish(FusedClustersList);
        std::cout<<"Printing fused coordinates"<<std::endl;
        // std::cout<<FusedClustersList.size<<" Clusters"<<std::endl;
    }

    void camera_callback(const sensor_msgs::PointCloudConstPtr& CameraClusters ) 
    {
        CameraClusterList.header.stamp = ros::Time::now();
        // CameraCluster.header.frame_id = LidarSource;
        CameraClusterList.size = 0;
        CameraClusterList.ConeCoordinates.clear();
        for (int index = 1; index <= ((*CameraClusters).channels)[0].values[0] ;index++){
            auto Cone = ((*CameraClusters).channels)[index];
          if (Cone.values[0] > 0.3){
            // std::cout <<"1";
            perception::Coordinates CurrentCluster;
            CurrentCluster.x = (Cone.values[1] + Cone.values[3])/2;
            CurrentCluster.y = Cone.values[2];
            CurrentCluster.z = 0;
            CurrentCluster.colour = ((Cone.name).compare("blue_cone") == 0) ? 1:0 ;
            // std::cout<<CurrentCluster.colour;
            CameraClusterList.ConeCoordinates.push_back(CurrentCluster);
            CameraClusterList.size ++;
          }   
        }
    
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

