#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
//ransac 
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>

//typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>

class ProgMorphFilter
{
public:
    ProgMorphFilter()
    {
        //ProgMorphFilterHandle{};
        Ground = ProgMorphFilterHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("Ground", 10);
        PointCloud = ProgMorphFilterHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("ConeCloud", 10);
        Subscribe = ProgMorphFilterHandle.subscribe("VoxelCloud", 10, &ProgMorphFilter::callback, this);
        //Subscribe = ProgMorphFilterHandle.subscribe("/velodyne_points", 10, &ProgMorphFilter::callback, this);
        //timer(ProgMorphFilterHandle.createTimer(ros::Duration(0.1), &ProgMorphFilter::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
        
        if(pt_cloud->size()==0){
            std::cout<<"No points found"<<std::endl;
            Ground.publish(*pt_cloud);
            PointCloud.publish(*pt_cloud);
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane (new pcl::PointCloud<pcl::PointXYZ> ), filtered_cloud  (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointIndicesPtr ground (new pcl::PointIndices);
        // Create the filtering object
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
        pmf.setInputCloud (pt_cloud);
        pmf.setMaxWindowSize (20);
        pmf.setSlope (1.0f);
        pmf.setInitialDistance (0.5f);
        pmf.setMaxDistance (3.0f);
        pmf.extract (ground->indices);
        
        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (pt_cloud);
        extract.setIndices (ground);
        extract.filter (*ground_plane);
        Ground.publish(*ground_plane);

        // Extract non-ground returns
        extract.setNegative (true);
        extract.filter (*filtered_cloud);
        PointCloud.publish(*filtered_cloud);
        std::cout<<"Publishing Ground"<<std::endl;
    }

private:
    ros::NodeHandle ProgMorphFilterHandle;        // ROS node handle
    ros::Publisher PointCloud;
    ros::Publisher Ground;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ProgMorphFilter");
    ProgMorphFilter node;
    ros::spin();
    return 0;
}
