#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
//ransac 
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>
//remove Nan
#include <pcl/filters/filter.h>

//typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>

class GroundRemoval
{
public:
    GroundRemoval()
    {
        //GroundRemovalHandle{};
        Ground = GroundRemovalHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("Ground", 10);
        PointCloud = GroundRemovalHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("ConeCloud", 10);
        // Subscribe = GroundRemovalHandle.subscribe("rslidar_points", 10, &GroundRemoval::callback, this);
        Subscribe = GroundRemovalHandle.subscribe("/velodyne_points", 10, &GroundRemoval::callback, this);
        //timer(GroundRemovalHandle.createTimer(ros::Duration(0.1), &GroundRemoval::main_loop, this));
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
        
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.13);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane (new pcl::PointCloud<pcl::PointXYZ> ), filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_plane_nan (new pcl::PointCloud<pcl::PointXYZ> ), filtered_cloud_nan (new pcl::PointCloud<pcl::PointXYZ>);
        seg.setInputCloud (pt_cloud);
        seg.segment (*inliers, *coefficients);
        ground_plane->is_dense = "true";
        filtered_cloud->is_dense = "true";
        ground_plane_nan->is_dense = "false";
        filtered_cloud_nan->is_dense = "false";
        std::vector<int> ind;
        if (inliers->indices.size () != 0){
            //exctract ground
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud (pt_cloud);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*ground_plane_nan);
            pcl::removeNaNFromPointCloud(*ground_plane_nan, *ground_plane, ind);
            Ground.publish(*ground_plane);

            //extract rest
            extract.setNegative (true);
            extract.filter (*filtered_cloud_nan);
            pcl::removeNaNFromPointCloud(*filtered_cloud_nan, *filtered_cloud,ind);
            PointCloud.publish(*filtered_cloud);
        }
        else{
            std::cout<<"No plane found"<<std::endl;
            Ground.publish(*ground_plane);
            PointCloud.publish(*pt_cloud);
        }
        std::cout<<"Publishing pointcloud"<< std::endl;
        //delete pt_cloud, coefficients, inliers, ground_plane;
    }

private:
    ros::NodeHandle GroundRemovalHandle;        // ROS node handle
    ros::Publisher PointCloud;
    ros::Publisher Ground;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "GroundRemoval");
    GroundRemoval node;
    ros::spin();
    return 0;
}
