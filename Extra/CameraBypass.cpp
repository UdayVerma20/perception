#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>
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


class CameraBypass
{
public:
    CameraBypass()
    {
        //CameraBypassHandle{};
        PcPub = CameraBypassHandle.advertise<clustering::CoordinateList>("Clusters", 10);
        PcPubPc = CameraBypassHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("FusedCoordinatesCLoud", 10);
        LidarSubscriber = CameraBypassHandle.subscribe("Clusters_temp", 10, &CameraBypass::lidar_callback, this);
        LidarPcSubscriber = CameraBypassHandle.subscribe("Clusters_PointCloud", 10, &CameraBypass::pc_callback, this);
        //timer(CameraBypassHandle.createTimer(ros::Duration(0.1), &CameraBypass::main_loop, this));
    }

    void lidar_callback(const clustering::CoordinateList& LidarClusters ) 
    {        
        PcPub.publish(LidarClusters);
        cout<<"Printing Bypassed coordinates"<<endl;
        // cout<<BypassedClustersList.size<<" Clusters"<<endl;
    }
    void pc_callback(const pcl::PointCloud<pcl::PointXYZI>& LidarClustersPc ) 
    {        
        PcPubPc.publish(LidarClustersPc);
        cout<<"Printing Bypassed coordinates PointCloud"<<endl;
        // cout<<BypassedClustersList.size<<" Clusters"<<endl;
    }

private:
    ros::NodeHandle CameraBypassHandle;        // ROS node handle
    ros::Publisher PcPub;
    ros::Publisher PcPubPc;
    ros::Subscriber LidarSubscriber;       // ROS subscriber for a topic
    ros::Subscriber LidarPcSubscriber; 
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CameraBypass");
    CameraBypass node;
    ros::spin();
    return 0;
}

