
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
//ransac 
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>

//typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>

class VoxelDownsample
{
public:
    VoxelDownsample()
    {
        //VoxelDownsampleHandle{};
        PointCloud = VoxelDownsampleHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("VoxelCloud", 10);
        Subscribe = VoxelDownsampleHandle.subscribe("velodyne_points", 10, &VoxelDownsample::callback, this);
        //timer(VoxelDownsampleHandle.createTimer(ros::Duration(0.1), &VoxelDownsample::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel(new pcl::PointCloud<pcl::PointXYZ>);
        
        if(pt_cloud->size()==0){
            std::cout<<"No points found"<<std::endl;
            PointCloud.publish(*pt_cloud);
            return;
        }
        
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (pt_cloud);
        sor.setLeafSize (0.04f, 0.04f, 0.04f);
        sor.filter (*cloud_voxel);
        std::cout<<"Downsampling Cloud"<<std::endl<<std::endl<<std::endl<<std::endl<<std::endl;
        PointCloud.publish(*cloud_voxel);
        
    }

private:
    ros::NodeHandle VoxelDownsampleHandle;        // ROS node handle
    ros::Publisher PointCloud;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "VoxelDownsample");
    VoxelDownsample node;
    ros::spin();
    return 0;
}

