#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
//clustering 
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iomanip> // for setw, setfill
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>

//publish 
#include <pcl_ros/point_cloud.h>
#include <pcl/impl/point_types.hpp>

typedef pcl::PointXYZ PointType;

//ConeCheck
#define LidarHeight 0.2
#define MaxHeight 1//0.7
#define MinHeight 0.1 //0.2
#define MaxPoints 2000
#define MinPoints 10//200
#define MaxLen 1 //0.7
#define MaxWidth 0.6

class Clustering
{
public:
    Clustering()
    {
        //ClusteringHandle{};
        Clusters = ClusteringHandle.advertise<perception::CoordinateList>("Clusters", 10);
        Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZ>>("Clusters_PointCloud", 10);
        Subscribe = ClusteringHandle.subscribe("ConeCloud", 10, &Clustering::callback, this);
        //timer(ClusteringHandle.createTimer(ros::Duration(0.1), &GroundRemoval::main_loop, this));
    }

    void callback(const pcl::PointCloud<PointType>::Ptr& pt_cloud)
        //sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*msg,pcl_pc2);
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);

        perception::CoordinateList Cluster;
        Cluster.header.stamp = ros::Time::now();
        Cluster.header.frame_id = "rslidar";//"rslidar";
        std::cout<<"1";

        if (pt_cloud->size()!= 0){
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_pc (new pcl::PointCloud<pcl::PointXYZ>);
            cluster_pc->header.frame_id = "rslidar";//"rslidar";
            tree->setInputCloud (pt_cloud);
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointType> ec;
            ec.setClusterTolerance (MaxLen); // 2cm
            ec.setMinClusterSize (MinPoints);
            ec.setMaxClusterSize (MaxPoints);
            ec.setSearchMethod (tree);
            ec.setInputCloud (pt_cloud);
            ec.extract (cluster_indices);

            Cluster.size = cluster_indices.size();
            cluster_pc->width = cluster_indices.size();
            cluster_pc->height = 1;
            std::cout<<cluster_indices.size()<<std::endl;
            std::cout<<"2";
            for (const auto& cluster : cluster_indices)
            {
                //std::cout<<ind<<std::endl;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
                //cloud_cluster->height = cloud_cluster->width = 1;
                for (const auto& idx : cluster.indices) {
                    //std::cout << idx<<cluster<<std::endl;
                    cloud_cluster->push_back(pcl::PointXYZ((*pt_cloud)[idx].x, (*pt_cloud)[idx].y, (*pt_cloud)[idx].z));
                } 
                //Clusters_pc.publish(cloud_cluster);
                //ros::Duration(1).sleep();
                //std::cout<< cloud_cluster->size()<<std::endl;
                std::cout<<"3";
                perception::Coordinates CurrentCluster;
                CurrentCluster.x = 0;
                CurrentCluster.y= 0;
                float minheight = FLT_MAX;
                float maxheight = -FLT_MAX;
                float left = FLT_MAX;
                float right = -FLT_MAX;
                int avgcnt = 0;
                for (auto i : cloud_cluster->points){
                    //CurrentCluster.y += i.x;
                    //CurrentCluster.x -= i.y;
                    CurrentCluster.y += i.y;
                    CurrentCluster.x += i.x;
                    avgcnt++;
                    if (minheight > i.z){
                        minheight = i.z;
                    }
                    if (maxheight < i.z){
                        maxheight = i.z;
                    }
                    if (left > i.x){
                        left = i.x;
                    }
                    if (right < i.x){
                        right = i.x;
                    }
                }
                CurrentCluster.x/= avgcnt; CurrentCluster.y/=avgcnt;
                CurrentCluster.z = minheight;
                int dist_sq = pow(CurrentCluster.x, 2.0) + pow(CurrentCluster.y, 2.0) + pow(CurrentCluster.z, 2.0);
                int expected_points = (5905.69)/(dist_sq);
                //std::cout <<std::endl<< maxheight<<minheight<<std::endl;
                //std::cout <<pow(dist_sq,0.5)<<" "<< expected_points <<" "<< cloud_cluster->size()<<std::endl;
                //(height * width)/(8 * distance^2 * tan(vert.reso/2) * tan(hori.reso/2))
                
                //cone check (add gradient) 
                if (1/*(maxheight + LidarHeight < MaxHeight) //remove tall obj
                && (minheight + LidarHeight < MinHeight)  //remove floating obj
                && (right - left < MaxLen)  //remove wide obj
                && (cloud_cluster->size() <= expected_points)*/
                ){
                    Cluster.ConeCoordinates.push_back(CurrentCluster);
                    cluster_pc->push_back(pcl::PointXYZ(CurrentCluster.x, CurrentCluster.y, CurrentCluster.z));
                }
                else {
                    std::cout << "Not a cone" << std::endl;
                    Cluster.size--;
                    cluster_pc->width--;
                }
            }
            Clusters_pc.publish(cluster_pc);
            std::cout<<"4";
        }
        else{
            std::cout<<"No object found"<<std::endl;
            Cluster.size = 0;
            /*perception::Coordinates CurrentCluster;
            CurrentCluster.x = 0;
            CurrentCluster.y= 0;
            CurrentCluster.z= 0;
            Cluster.ConeCoordinates.push_back(CurrentCluster);*/
        }

        Clusters.publish(Cluster);
        std::cout<<"Publishing Cone Coordinates"<<std::endl;
    }


private:
    ros::NodeHandle ClusteringHandle;        // ROS node handle
    ros::Publisher Clusters;        // ROS publisher for JointState messages
    ros::Publisher Clusters_pc;
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Clustering");
    Clustering node;
    ros::spin();
    return 0;
}
