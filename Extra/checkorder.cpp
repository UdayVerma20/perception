
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>
#include <unistd.h>

#define deg 0.05f
//typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>

class CheckOrder
{
public:
    CheckOrder()
    {
        //CheckOrderHandle{};
        PointCloud = CheckOrderHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("OrderPc", 10);
        Subscribe = CheckOrderHandle.subscribe("rslidar_points", 10, &CheckOrder::callback, this);
        //timer(CheckOrderHandle.createTimer(ros::Duration(0.1), &CheckOrder::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr order_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        order_cloud->header.frame_id="rslidar";
        std::cout<<"Starting Pub"<<std::endl;//<<std::endl<<std::endl<<std::endl<<std::endl;
        int i = 1;
        for (auto point : pt_cloud->points){
            if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)){
                std::cout<<i <<" "<<point.y<<std::endl;
                i++;
                order_cloud->push_back(point);
                PointCloud.publish(*order_cloud);
                usleep(10000);
            }
        }
        
        
    }

private:
    ros::NodeHandle CheckOrderHandle;        // ROS node handle
    ros::Publisher PointCloud;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "CheckOrder");
    CheckOrder node;
    ros::spin();
    return 0;
}

