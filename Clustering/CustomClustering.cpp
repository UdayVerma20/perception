#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
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

// #include <pcl/impl/point_types.hpp>
// #include <pcl/filters/filter.h>

#define LidarSource "rslidar"

//ConeCheck
#define LidarHeight 0.27
#define MaxHeight 0.2
#define MinHeight 0.03
// #define MinHeight 0.1 //0.2
// #define MaxPoints 2000
#define MinPoints 7
// #define MaxLen 1 //0.7
#define MaxWidth 0.6

typedef pcl::PointXYZI PointType;

//thresholds
float EuclideanThreshold =  0.01;
float CentroidThreshold = 0.045;

struct IntensityCalc{
    float Value = 0;
    int size = 0;
};

class CustomCluster{
    public:
    int clustersize = 0;    // int clusterindex = 0;
    // long long Centre_x;
    // long long Centre_y;
    // long long Centre_z;
    PointType Avg;
    PointType Last;
    PointType Left;
    PointType Right;
    std::map<float,IntensityCalc> gradient;
    // float minheight;
    // float maxheight ;
    // float left;
    // float right ;
    // PointType PointsArr[11000];  //Don't stor points in vector, too slow

    CustomCluster(PointType Point){
        Avg = Point;
        Last = Point;
        Left = Point;
        Right = Point;
        // PointsArr[clustersize++] = Point;
        clustersize = 1;
        (gradient[(int)(Point.z*100)].Value)+= Point.intensity;
        (gradient[(int)(Point.z*100)].size)++;
    //     // Centre_x = Point.x;
    //     // Centre_y = Point.y;
    //     // Centre_z = Point.z;
    //     // minheight = maxheight = Point.z;
    //     // left = right = Point.x;
    }
};

int check_distance(PointType Cluster_Point, PointType point, float Threshold){
    float distance = pow((Cluster_Point.x  - point.x), 2) + pow((Cluster_Point.y  - point.y), 2)
     + pow((Cluster_Point.z  - point.z), 2);
    // if (distance < Threshold){std::cout<<Threshold<<" "<<distance<<" "<< (distance < Threshold)<<std::endl;}
    return (distance < Threshold);
}

class Clustering
{
public:
    Clustering()
    {
        //ClusteringHandle{};
        Clusters = ClusteringHandle.advertise<perception::CoordinateList>("Clusters", 1000);
        Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<PointType>>("Clusters_PointCloud", 1000);
        // All_Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<PointType>>("All_Clusters_PointCloud", 100);
        Subscribe = ClusteringHandle.subscribe("ConeCloud", 10, &Clustering::callback, this);
        //timer(ClusteringHandle.createTimer(ros::Duration(0.1), &GroundRemoval::main_loop, this));
    }

    void callback(const pcl::PointCloud<PointType>::Ptr& pt_cloud)
        //sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*msg,pcl_pc2);
        // pcl::PointCloud<PointType>::Ptr pt_cloud(new pcl::PointCloud<PointType>);
        // pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);

        perception::CoordinateList Cluster;
        Cluster.header.stamp = ros::Time::now();
        Cluster.header.frame_id = LidarSource;
        Cluster.size = 0;
        /*perception::Coordinates CurrentCluster;
        CurrentCluster.x = 0;
        CurrentCluster.y= 0;
        CurrentCluster.z= 0;
        Cluster.ConeCoordinates.push_back(CurrentCluster);*/
        // std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
        std::vector<CustomCluster> Clusters_Vector;
        CustomCluster StdCluster(PointType(0.f));
        Clusters_Vector.push_back(StdCluster);

        if (pt_cloud->size()!= 0){
            pcl::PointCloud<PointType>::Ptr cluster_pc (new pcl::PointCloud<PointType>);
            // pcl::PointCloud<PointType>::Ptr all_cluster_pc (new pcl::PointCloud<PointType>);
            // all_cluster_pc->header.frame_id = LidarSource;
            //cluster_pc->push_back(PointType(0,0,0,0));
            cluster_pc->header.frame_id = LidarSource;

            // std::cout<<cluster_indices.size()<<std::endl;
            // std::cout<<"2";
            // std::vector<int> ind;
            // pcl::removeNaNFromPointCloud(*pt_cloud, *pt_cloud, ind);
            // int pointno = 1;
            for (PointType point : pt_cloud->points) { 
                // std::cout<<"BBBB"<<pointno++<<std::endl;
                if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)){
                    int found_cluster = 0;
                    for (int index = 0; index < Clusters_Vector.size(); index++){
                        CustomCluster Iter_Cluster = Clusters_Vector[index];
                        if (
                            check_distance(Clusters_Vector[index].Last, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Left, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Right, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Avg, point, CentroidThreshold)){
                            // before increment
                            // Clusters_Vector[index].Avg.x = (Iter_Cluster.Avg.x + (point.x/Iter_Cluster.clustersize))/(1 + (1/Iter_Cluster.clustersize));
                            // Clusters_Vector[index].Avg.y = (Iter_Cluster.Avg.y + (point.y/Iter_Cluster.clustersize))/(1 + (1/Iter_Cluster.clustersize));
                            // Clusters_Vector[index].Avg.z = //(Iter_Cluster.Avg.z < point.z) ? Iter_Cluster.Avg.z : point.z;
                            // (Iter_Cluster.Avg.z + (point.z/Iter_Cluster.clustersize))/(1 + (1/Iter_Cluster.clustersize));
                            //Clusters_Vector[index].clustersize++;
                            
                            //after increment
                            int current_size = (Clusters_Vector[index].clustersize++);
                            double inv_current_size = (double)1/(current_size+1);

                            Clusters_Vector[index].Last = point;
                            Clusters_Vector[index].Left =  (Iter_Cluster.Left.y >= point.y) ? point: Clusters_Vector[index].Left ;
                            Clusters_Vector[index].Right = (Iter_Cluster.Right.y <= point.y) ? point: Clusters_Vector[index].Right;

                            Clusters_Vector[index].Avg.x = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.x + (double)(point.x*inv_current_size);
                            Clusters_Vector[index].Avg.y = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.y + (double)(point.y*inv_current_size);
                            Clusters_Vector[index].Avg.z = //(Iter_Cluster.Avg.z < point.z) ? Iter_Cluster.Avg.z : point.z;
                            (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.z + (double)(point.z*inv_current_size);
                            
                            (Clusters_Vector[index].gradient[(int)(point.z*100)].Value)+= point.intensity;
                            (Clusters_Vector[index].gradient[(int)(point.z*100)].size)++;
                            //Clusters_Vector[index].clustersize++;
                            // Iter_Cluster.PointsArr.push_back(point);
                            // std::cout<<"A"<<i<<std::endl;
                            // Iter_Cluster.Centre_x += point.x;
                            // Iter_Cluster.Centre_y += point.y;
                            // Iter_Cluster.Centre_z += point.z;
                            // int non0size = 1>Iter_Cluster.clustersize?1:Iter_Cluster.clustersize;
                            // Clusters_Vector[index].PointsArr[current_size] = point;
                            found_cluster++;
                            // std::cout<<found_cluster<<"a";
                            // all_cluster_pc->push_back(point);
                            // std::cout<<Iter_Cluster.clustersize;
                            // Clusters_Vector[index].clustersize++; 
                            // std::cout<<Iter_Cluster.clustersize;
                            // if (Iter_Cluster.minheight > point.z){
                            //     Iter_Cluster.minheight = point.z;
                            // }
                            // if (Iter_Cluster.maxheight < point.z){
                            //     Iter_Cluster.maxheight = point.z;
                            // }
                            // if (Iter_Cluster.left > point.x){
                            //     Iter_Cluster.left = point.x;
                            // }
                            // if (Iter_Cluster.right < point.x){
                            //     Iter_Cluster.right = point.x;
                            // }
                            // std::cout<<index<<std::endl;
                            // Clusters_Vector[index] = Iter_Cluster;
                            break;
                        }
                        //std::cout<<check_distance(Iter_Cluster, point)<<std::endl;
                    }
                    // std::cout<<found_cluster<<std::endl;
                    if (!found_cluster){
                        CustomCluster NewCluster(point);
                        // NewCluster.PointsArr.push_back(point);
                        Clusters_Vector.push_back(NewCluster);
                    
                    }
                }
            }

            // std::cout<<Clusters_Vector.size()<<std::endl;
            cluster_pc->width = Clusters_Vector.size();
            cluster_pc->height = 1;
            cluster_pc->push_back(PointType(0.f));
            for (int index = 0; index < Clusters_Vector.size(); index++){
                CustomCluster Iter_Cluster = Clusters_Vector[index];
                int dist_sq = pow(Iter_Cluster.Avg.x, 2.0) + pow(Iter_Cluster.Avg.y, 2.0) + pow(Iter_Cluster.Avg.z, 2.0);
                int expected_points = (3000)/(dist_sq + 1); //remove dist =0
                //(height * width)/(8 * distance^2 * tan(vert.reso/2) * tan(hori.reso/2))
                //std::cout <<std::endl<< maxheight<<minheight<<std::endl;
                // std::cout <<pow(dist_sq,0.5)<<" "<< expected_points <<" "<< Iter_Cluster.size<<std::endl;
                int curr_colour = 1;
                int dip = 0;
                for (auto it = Iter_Cluster.gradient.begin()++; it!=Iter_Cluster.gradient.end(); ++it){
                    auto prev_it = it;
                    // std::cout<<"1";
                    --prev_it;
                    std::cout<<it->first<<" "<<it->second.Value/it->second.size<< " " << it->second.size<<std::endl;
                    if((it->second.Value)/(it->second.size) > 2.5*(prev_it->second.Value)/(prev_it->second.size)){
                        if (dip == 1){
                            curr_colour =0;
                            break;
                        }
                        // else{
                        //     curr_colour = 0;
                        // }
                    }
                    if(2.5*(it->second.Value)/(it->second.size) < (prev_it->second.Value)/(prev_it->second.size)){
                        // curr_colour =0;
                        dip = 1;
                    }
                }
                // cone check (add gradient) 
                if (
                // 1 
                (Iter_Cluster.Avg.z + LidarHeight < MaxHeight) //remove floating obj
                && (Iter_Cluster.Avg.z + LidarHeight > MinHeight)
                && (Iter_Cluster.clustersize < expected_points)
                // // && (Iter_Cluster.clustersize > 0.13 * expected_points)
                && (Iter_Cluster.Right.y - Iter_Cluster.Left.y < MaxWidth)
                && (Iter_Cluster.clustersize > MinPoints)
                // && (curr_colour == 0)
                ){
                    Cluster.size++;
                    perception::Coordinates CurrentCluster;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                    CurrentCluster.colour = curr_colour;
                    std::cout << "Cone " <<Iter_Cluster.clustersize<<" "<<curr_colour<<" "
                    <<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z<<" "<<std::endl; //<<" "<<dist_sq*Iter_Cluster.clustersize<< std::endl;
                    Cluster.ConeCoordinates.push_back(CurrentCluster);
                    cluster_pc->push_back(Iter_Cluster.Avg);
                
                }
                // else {
                //     //std::cout << "Not a cone" << std::endl;
                //     //Cluster.size--;
                //     //cluster_pc->width--;
                // }
            }
            // std::cout<<"3";
            Clusters_pc.publish(*cluster_pc);
            // All_Clusters_pc.publish(*all_cluster_pc);
            // std::cout<<"4";
            
        }
        else{
            std::cout<<"No object found"<<std::endl;
            //Cluster.size = 0;
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
    // ros::Publisher All_Clusters_pc;
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
