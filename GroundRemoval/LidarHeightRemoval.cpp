#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>

//plane removal
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>

//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

//publish 
#include <pcl_ros/point_cloud.h>
// #include <pcl_ros/point_cloud.h>

//remove Nan
#include <pcl/filters/filter.h>
#include "pcl-1.10/pcl/filters/impl/filter.hpp"
#include <pcl/filters/passthrough.h>

#define LidarHeight 0.285 //0.2
#define Threshold 0.01
#define LidarSource "rslidar"
#define LidarTopic "rslidar_points"

typedef pcl::PointXYZI PointType;

class LidarHeightRemoval
{
public:
    LidarHeightRemoval()
    {
        //LidarHeightRemovalHandle{};
        Ground = LidarHeightRemovalHandle.advertise<pcl::PointCloud<PointType>>("Ground", 10);
        PointCloud = LidarHeightRemovalHandle.advertise<pcl::PointCloud<PointType>>("ConeCloud", 10);
        Subscribe = LidarHeightRemovalHandle.subscribe(LidarTopic, 10, &LidarHeightRemoval::callback, this);
        //timer(LidarHeightRemovalHandle.createTimer(ros::Duration(0.1), &LidarHeightRemoval::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr ground(new pcl::PointCloud<PointType>);
        pcl::PointCloud<PointType>::Ptr cone_cloud(new pcl::PointCloud<PointType>);
        // pcl::PointCloud<PointType>::Ptr ground_nan(new pcl::PointCloud<PointType>);
        // pcl::PointCloud<PointType>::Ptr cone_cloud_nan(new pcl::PointCloud<PointType>);
        ground->header.frame_id = LidarSource;
        // ground_nan->header.frame_id = LidarSource;
        ground->is_dense = "false";
        cone_cloud->header.frame_id = LidarSource;
        // cone_cloud_nan->header.frame_id = LidarSource;
        cone_cloud->is_dense = "false";
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);
        // float minimumx = INT_MAX;
        // float minimumy = INT_MAX;
        // float minimumz = INT_MAX;
        // float maximumx = INT_MIN;
        // float maximumy = INT_MIN;
        // float maximumz = INT_MIN;
        for (auto i : cloud->points) {
            // std::cout<<i.x<<" "<<i.y<<" "<<i.z<<std::endl;
        //      if(i.z >maximumz){
        //          maximumz = i.z;
        //      }
        //      if(i.y >maximumy){
        //          maximumy = i.y;
        //      }
        //      if(i.x >maximumx){
        //          maximumx = i.x;
        //      }
        //      if(i.x < minimumx){
        //          minimumx = i.x;
        //      }
        //      if(i.y < minimumy){
        //          minimumy = i.y;
        //      }
        //      if(i.z < minimumz){
        //          minimumz = i.z;
        //      }}
        //    std::cout << minimumx <<" " <<maximumx <<" "<<minimumy <<" " <<maximumy <<" "<<minimumz <<" " <<maximumz <<" "<<std::endl;
            if ((i.z + LidarHeight) < Threshold || (i.x*i.x + i.y*i.y > 60)
          || (i.x*i.x + i.y*i.y < 6) || (abs(i.y) > 1) || (i.z + LidarHeight) > 0.6
            ){
                ground->push_back(i);
            }
            else{
                cone_cloud->push_back(i);
            }
        }
        //std::cout<<mini<<std::endl;
        // std::vector<int> ind;
        // pcl::removeNaNFromPointCloud(*ground, *ground, ind);
        // pcl::PassThrough<PointType> pass;
        // pass.setInputCloud (ground);
        // pass.setFilterFieldName ("x");
        // pass.setFilterLimits (-10, 10);
        // //pass.setNegative (true);
        // pass.filter (*ground);
        // pass.setInputCloud (ground);
        // pass.setFilterFieldName ("y");
        // pass.setFilterLimits (-10, 10);
        // //pass.setNegative (true);
        // pass.filter (*ground);
        // //pcl::removeNaNNormalsFromPointCloud(*ground, *ground, ind);
        ground->is_dense = "true";
        Ground.publish(*ground);        
        // pcl::removeNaNFromPointCloud(*cone_cloud, *cone_cloud, ind);
        // pass.setInputCloud (cone_cloud);
        // pass.setFilterFieldName ("x");
        // pass.setFilterLimits (-10, 10);
        // //pass.setNegative (true);
        // pass.filter (*cone_cloud);
        // pass.setInputCloud (cone_cloud);
        // pass.setFilterFieldName ("y");
        // pass.setFilterLimits (-10, 10);
        // //pass.setNegative (true);
        // pass.filter (*cone_cloud);
        // //pcl::removeNaNNormalsFromPointCloud(*cone_cloud, *cone_cloud, ind);
        cone_cloud->is_dense = "true";
        PointCloud.publish(*cone_cloud);
        std::cout<<"Publishing Cone Cloud" <<std::endl;
        //<<ground->size()<<" "<<cone_cloud->size()

    }


private:
    ros::NodeHandle LidarHeightRemovalHandle;        // ROS node handle
    ros::Publisher PointCloud;
    ros::Publisher Ground;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LidarHeightRemoval");
    LidarHeightRemoval node;
    ros::spin();
    return 0;
}

