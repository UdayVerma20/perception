#include <cmath>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <clustering/CoordinateList.h>
#include <clustering/Coordinates.h>

using namespace std;
#define LidarSource "rslidar"
#define LidarHeight 0.27 
#define Theta 0.0174533*0
#define margin 0.1
sensor_msgs::PointCloudConstPtr CurrentCameraCluster;
clustering::CoordinateList CameraClusterList;
clustering::CoordinateList FusedClustersList;

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
        FusedCoordinates = SensorFusionHandle.advertise<clustering::CoordinateList>("Clusters", 10);
        FusedCoordinatesPc = SensorFusionHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("FusedCoordinatesCLoud", 10);
        LidarSubscriber = SensorFusionHandle.subscribe("Clusters_temp", 10, &SensorFusion::lidar_callback, this);
        CameraSubscriber = SensorFusionHandle.subscribe("detect_info", 10, &SensorFusion::camera_callback, this);
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
        pcl::PointCloud<pcl::PointXYZ>::Ptr fused_pc (new pcl::PointCloud<pcl::PointXYZ>);
        fused_pc->header.frame_id = LidarSource;
        vector<bool> CameraBoxTook(((*CurrentCameraCluster).channels)[0].values[0],1);
        for (int index = 0; index < sortedClusters.size(); index++){
            clustering::Coordinates LidarCluster = sortedClusters[index];
            LidarCluster.x += 1.5;
            LidarCluster.y -= 0.06;
            LidarCluster.z -= 0.59;

            float Lidar_Rotated_Y = LidarCluster.y*cos(Theta) - LidarCluster.z*sin(Theta);
            float Lidar_Rotated_Z = LidarCluster.y*sin(Theta) + LidarCluster.z*cos(Theta);
            LidarCluster.y = Lidar_Rotated_Y;
            LidarCluster.z = Lidar_Rotated_Z;
            // zc = x, xc = -y, yc = -z
            float Lidar_U = 525.0 * (- LidarCluster.y)/(LidarCluster.x) + 360.5;
            float Lidar_V = 525.0 * (- LidarCluster.z)/(LidarCluster.x) + 295.5; 
        
            for (int nested_index = 1; nested_index <= ((*CurrentCameraCluster).channels)[0].values[0] ;nested_index++){
                auto Camera_Cone = ((*CurrentCameraCluster).channels)[nested_index];
                if ((Camera_Cone.values[0] > 0.3)
                 && (CameraBoxTook[nested_index])
                 && ((Lidar_U>= (1+margin)*Camera_Cone.values[1] - margin*Camera_Cone.values[3]) && (Lidar_U<= (1+margin)*Camera_Cone.values[3] - margin*Camera_Cone.values[1]))
                 && ((Lidar_V>= (1+margin)*Camera_Cone.values[2] - margin*Camera_Cone.values[4]) && (Lidar_V<= (1+margin)*Camera_Cone.values[4] - margin*Camera_Cone.values[2]))
                 ){
                    clustering::Coordinates CurrentCluster;
                    CurrentCluster.x = LidarCluster.x;
                    CurrentCluster.y = LidarCluster.y;
                    CurrentCluster.z = LidarCluster.z ;
                    CurrentCluster.colour = ((Camera_Cone.name).compare("blue_cone") == 0) ? 1:0; // blue = 1
                    fused_pc->push_back(pcl::PointXYZ(sortedClusters[index].x, sortedClusters[index].y, sortedClusters[index].z));
                    FusedClustersList.ConeCoordinates.push_back(CurrentCluster);
                    (FusedClustersList.size)++;
                    CameraBoxTook[nested_index]=0;
                    break;
                }   
            }
        }
        FusedCoordinatesPc.publish(*fused_pc);
        FusedCoordinates.publish(FusedClustersList);
        cout<<"Printing fused coordinates"<<endl;
    }

    void camera_callback(const sensor_msgs::PointCloudConstPtr& CameraClusters ) 
    {
        CurrentCameraCluster = CameraClusters;    
    }


private:
    ros::NodeHandle SensorFusionHandle;
    ros::Publisher FusedCoordinates;
    ros::Publisher FusedCoordinatesPc;
    ros::Subscriber LidarSubscriber;
    ros::Subscriber CameraSubscriber;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SensorFusion");
    SensorFusion node;
    ros::spin();
    return 0;
}

