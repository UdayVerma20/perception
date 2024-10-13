#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <math.h>
#include <iomanip> // for setw, setfill
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/filter.h>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/filters/voxel_grid.h>
#include <perception/CoordinateList.h>
#include <perception/Coordinates.h>
#include <vector>
#include <cstdlib>
// #include <pcl/impl/point_types.hpp>
// #include <pcl/filters/filter.h>
#include<bits/stdc++.h>
#include <pcl_ros/point_cloud.h>

#define heightlidar 0.25 //0.188
#define ringlength 0.4 //1.5
#define sectorangle M_PI/768 //sector angle has to be a multiple of pi/6
#define startatring 0
#define planespreadtillring 50
#define ringthreshold 0.1 //0.13
#define lidarframe "rslidar" //"rslidar"

int markerid = 0;

//ConeCheck
#define MaxHeight 0.4
#define MinHeight 0.05
// #define MinHeight 0.1 //0.2
// #define MaxPoints 2000
#define MinPoints 10
// #define MaxLen 1 //0.7
#define MaxWidth 0.4

// typedef pcl::PointXYZI pcl::PointXYZI;

//thresholds
float EuclideanThreshold =  0.4;
float CentroidThreshold = 0.4;

// struct IntensityCalc{
//     float Value = 0;
//     int size = 0;
// };

ros::Publisher pub;
ros::Publisher marker_pub;
std::unordered_map<int, pcl::PointXYZI> bin_min_points;
std::unordered_map<int, std::vector<float>> line;

pcl::PointXYZI lidarpoint;

void
propogate (pcl::PointXYZI lastpoint, int sector, int ringcount, int numberofpoints){
  if(ringcount>=planespreadtillring){
    if(lastpoint.x==lidarpoint.x && lastpoint.y==lidarpoint.y && lastpoint.y==lidarpoint.y){
      lastpoint.x = lidarpoint.x + planespreadtillring*ringlength*cos((sector*sectorangle)+M_PI/6);
      lastpoint.y = lidarpoint.y + planespreadtillring*ringlength*sin((sector*sectorangle)+M_PI/6);
    }
    line[sector] = {lastpoint.x, lastpoint.y, lastpoint.z};
    return;
  }
  if(bin_min_points.find(ringcount*(2*M_PI/(3*sectorangle)) + sector)!=bin_min_points.end()){
    numberofpoints++;
    float avgpointx = ((float)(lastpoint.x*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].x))/((float)(numberofpoints + 1));
    float avgpointy = ((float)(lastpoint.y*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].y))/((float)(numberofpoints + 1));
    float avgpointz = ((float)(lastpoint.z*(numberofpoints) + bin_min_points[ringcount*(2*M_PI/(3*sectorangle)) + sector].z))/((float)(numberofpoints + 1));
    lastpoint.x = avgpointx; lastpoint.y = avgpointy; lastpoint.z = avgpointz;
  }

  propogate(lastpoint, sector, ringcount+1, numberofpoints);
}

void visualise(){
  visualization_msgs::MarkerArray visualisationarray;
  //making rings
  // for(int i=0; i<planespreadtillring; i++){
  //   visualization_msgs::Marker ringmarker;
  //   ringmarker.header.frame_id = lidarframe;
  //   ringmarker.header.stamp = ros::Time::now();
  //   ringmarker.ns = "basic_shapes";
  //   ringmarker.id = markerid++;
  //   ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
  //   ringmarker.action = visualization_msgs::Marker::ADD;
  //   ringmarker.lifetime = ros::Duration(0.1);

  //   ringmarker.pose.orientation.w=1.0;
  //   ringmarker.scale.x = 0.005;
  //   ringmarker.color.r = 1.0;
  //   ringmarker.color.a = 1.0;
  //   for(int j=0; j<13; j++){
  //     geometry_msgs::Point ringpoint;
  //     ringpoint.x = i*ringlength*sin(j*M_PI/18 + M_PI/6);
  //     ringpoint.y = -i*ringlength*cos(j*M_PI/18 + M_PI/6);
  //     ringpoint.z = -heightlidar;
  //     ringmarker.points.push_back(ringpoint);
  //   }
  //   visualisationarray.markers.push_back(ringmarker);
  // }

  // //making sectors
  // for(int i=0; i<=2*M_PI/(3*sectorangle); i++){
  //   visualization_msgs::Marker ringmarker;
  //   ringmarker.header.frame_id = lidarframe;
  //   ringmarker.header.stamp = ros::Time::now();
  //   ringmarker.ns = "basic_shapes";
  //   ringmarker.id = markerid++;
  //   ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
  //   ringmarker.action = visualization_msgs::Marker::ADD;
  //   ringmarker.lifetime = ros::Duration(0.1);

  //   ringmarker.pose.orientation.w=1.0;
  //   ringmarker.scale.x = 0.005;
  //   ringmarker.color.r = 1.0;
  //   ringmarker.color.a = 1.0;
   
  //   geometry_msgs::Point sectorpoint1;
  //   sectorpoint1.x = 0;sectorpoint1.y=0;sectorpoint1.z=-heightlidar;
  //   ringmarker.points.push_back(sectorpoint1);
  //   geometry_msgs::Point sectorpoint2;
  //   sectorpoint2.x = planespreadtillring*ringlength*sin(i*sectorangle + M_PI/6);sectorpoint2.y=-planespreadtillring*ringlength*cos(i*sectorangle + M_PI/6);sectorpoint2.z=-heightlidar;
  //   ringmarker.points.push_back(sectorpoint2);
  //   visualisationarray.markers.push_back(ringmarker);
  // }

  // making planes
  for(int i=0; i<2*M_PI/(3*sectorangle); i++){
    if(line.find(i)==line.end()) continue;
    visualization_msgs::Marker ringmarker;
    ringmarker.header.frame_id = lidarframe;
    ringmarker.header.stamp = ros::Time::now();
    ringmarker.ns = "basic_shapes";
    ringmarker.id = markerid++;
    ringmarker.type = visualization_msgs::Marker::LINE_STRIP;
    ringmarker.action = visualization_msgs::Marker::ADD;
    ringmarker.lifetime = ros::Duration(0.1);

    ringmarker.pose.orientation.w=1.0;
    ringmarker.scale.x = 0.005;
    ringmarker.color.r = 1.0;
    ringmarker.color.a = 1.0;
   
    geometry_msgs::Point sectorpoint1;
    sectorpoint1.x = lidarpoint.x; sectorpoint1.y=lidarpoint.y; sectorpoint1.z=lidarpoint.z;
    ringmarker.points.push_back(sectorpoint1);
    geometry_msgs::Point sectorpoint2;
    sectorpoint2.x = planespreadtillring*ringlength*(line[i][0] - lidarpoint.x) + lidarpoint.x ;sectorpoint2.y= planespreadtillring*ringlength*(line[i][1] - lidarpoint.y) + lidarpoint.y  ; planespreadtillring*ringlength*(line[i][2] - lidarpoint.z) + lidarpoint.z ;
    ringmarker.points.push_back(sectorpoint2);
    visualisationarray.markers.push_back(ringmarker);
  }

  ROS_INFO("Visualising to /visualization_marker");
  marker_pub.publish(visualisationarray);
}

class CustomCluster{
    public:
    int clustersize = 0;    // int clusterindex = 0;
    // long long Centre_x;
    // long long Centre_y;
    // long long Centre_z;
    pcl::PointXYZI Avg;
    pcl::PointXYZI Last;
    pcl::PointXYZI Left;
    pcl::PointXYZI Right;
    // std::map<float,IntensityCalc> gradient;
    pcl::PointXYZI minheight;
    pcl::PointXYZI maxheight ;
    // float left;
    // float right ;
    // pcl::PointXYZI PointsArr[11000];  //Don't stor points in vector, too slow

    CustomCluster(pcl::PointXYZI Point){
        Avg = Point;
        Last = Point;
        Left = Point;
        Right = Point;
        // PointsArr[clustersize++] = Point;
        clustersize = 1;
        // (gradient[(int)(Point.z*100)].Value)+= Point.intensity;
        // (gradient[(int)(Point.z*100)].size)++;
    //     // Centre_x = Point.x;
    //     // Centre_y = Point.y;
    //     // Centre_z = Point.z;
        minheight = maxheight = Point;
    //     // left = right = Point.x;
    }
};

int check_distance(pcl::PointXYZI Cluster_Point, pcl::PointXYZI point, float Threshold){
    float distance = pow((Cluster_Point.x  - point.x), 2.0) + pow((Cluster_Point.y  - point.y), 2.0)
     + pow((Cluster_Point.z  - point.z), 2.0);
    // if (distance < Threshold){std::cout<<Threshold<<" "<<distance<<" "<< (distance < Threshold)<<std::endl;}
    return (distance < Threshold);
}

class Clustering
{
public:
    Clustering(ros::NodeHandle ClusteringHandle)
    {
        //ClusteringHandle{};
        Clusters = ClusteringHandle.advertise<perception::CoordinateList>("Clusters", 1000);
        Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("Clusters_PointCloud", 1000);
        // pub = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ConeCloud", 1);
        // All_Clusters_pc = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("All_Clusters_PointCloud", 100);
        // Subscribe = ClusteringHandle.subscribe("ConeCloud", 10, &Clustering::callback, this);
        sub = ClusteringHandle.subscribe ("rslidar_points", 1, &Clustering::callback, this);
        //timer(ClusteringHandle.createTimer(ros::Duration(0.1), &GroundRemoval::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& input)
        //sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        // pcl::PCLPointCloud2 pcl_pc2;
        // pcl_conversions::toPCL(*msg,pcl_pc2);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudtomanipulate(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg (*input, *cloud);

  pcl::Indices indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloudtomanipulate, indices);
  // cloudtomanipulate->header.frame_id = "rslidar";
 
  bin_min_points.clear();
  line.clear();
  lidarpoint.x = 0.f; lidarpoint.y = 0.f; lidarpoint.z = -heightlidar;

  //segregating points into bins and placing them in bin_min_points and bin_all_points
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    // if((cloudtomanipulate->points)[index].y >2.0 || (cloudtomanipulate->points)[index].y<-2.0) continue;
    // //cutoff
    // if((cloudtomanipulate->points)[index].z >0.5) continue;

    int ring = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(ring>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;

    //this key starts from 0 from right direction and denotes the bin number. It increases as we move towards left or farther away from lidar
    int key = int((angle)/(sectorangle)) + int((2*M_PI*ring)/(3*sectorangle));

    //adding to bin_min_points
    if(bin_min_points.find(key)!=bin_min_points.end()){
      if((cloudtomanipulate->points)[index].z<bin_min_points[key].z){
        bin_min_points[key] = (cloudtomanipulate->points)[index];
      }
    }else{
      bin_min_points[key] = (cloudtomanipulate->points)[index];
    }
  }
 
  //propogating the planes in all the bins
  for(int i=int(2*M_PI*startatring/(3*sectorangle)); i<int(2*M_PI*(startatring+1)/(3*sectorangle)); i++){
    propogate(lidarpoint, i, startatring, 0);
  }

  visualise();

  //removing points in the viscinity of the planes
  pcl::PointCloud<pcl::PointXYZI>::Ptr nonground(new pcl::PointCloud<pcl::PointXYZI>);
  nonground->header.frame_id = lidarframe;
  for(int index=0; index<cloudtomanipulate->points.size(); index++){

    //trimming view area
    // if((cloudtomanipulate->points)[index].y >2.0 || (cloudtomanipulate->points)[index].y<-2.0) continue;
    // if((cloudtomanipulate->points)[index].z >0.5) continue;
    int ring = pow(pow((cloudtomanipulate->points)[index].x, 2)+pow((cloudtomanipulate->points)[index].y, 2), 0.5)/ringlength;
    if(ring>=planespreadtillring) continue;
    float angle = atan2((cloudtomanipulate->points)[index].x, -(cloudtomanipulate->points)[index].y) - M_PI/6;
    if(angle >=2*M_PI/3|| angle <0) continue;
    int key = int((angle)/(sectorangle));
    if(line.find(key)==line.end()) continue;
    float denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
    float term1 = pow((line[key][1] - lidarpoint.y)*((cloudtomanipulate->points)[index].z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*((cloudtomanipulate->points)[index].y-lidarpoint.y), 2);
    float term2 = pow((line[key][2] - lidarpoint.z)*((cloudtomanipulate->points)[index].x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*((cloudtomanipulate->points)[index].z-lidarpoint.z), 2);
    float term3 = pow((line[key][0] - lidarpoint.x)*((cloudtomanipulate->points)[index].y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*((cloudtomanipulate->points)[index].x-lidarpoint.x), 2);
    float distance = (pow(term1 + term2 + term3, 0.5))/(denominator);

    if(distance>ringthreshold) nonground->push_back((cloudtomanipulate->points)[index]);

  }

  // Publish the data.
  ROS_INFO("Publishing to nogroundcloud");
  // int count=0;
  // for (auto i:line){
  //   count++;
  //   // std::cout << i.first << ": " << i.second[0] << " " << i.second[1] << " "<< i.second[2] << std::endl;
  // }
  // std::cout << count << std::endl;


  
  // ros::Rate rate(4);
  pub.publish (*nonground);
 
        perception::CoordinateList Cluster;
        Cluster.header.stamp = ros::Time::now();
        Cluster.header.frame_id = lidarframe;
        Cluster.size = 0;
        /*perception::Coordinates CurrentCluster;
        CurrentCluster.x = 0;
        CurrentCluster.y= 0;
        CurrentCluster.z= 0;
        Cluster.ConeCoordinates.push_back(CurrentCluster);*/
        std::vector<CustomCluster> Clusters_Vector;
        CustomCluster StdCluster(pcl::PointXYZI(0.f));
        Clusters_Vector.push_back(StdCluster);

        if (nonground->size()!= 0){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_pc (new pcl::PointCloud<pcl::PointXYZI>);
            // pcl::PointCloud<pcl::PointXYZI>::Ptr all_cluster_pc (new pcl::PointCloud<pcl::PointXYZI>);
            // all_cluster_pc->header.frame_id = lidarframe;
            //cluster_pc->push_back(pcl::PointXYZI(0,0,0,0));
            cluster_pc->header.frame_id = lidarframe;

            // std::cout<<cluster_indices.size()<<std::endl;
            // std::cout<<"2";
            // std::vector<int> ind;
            // pcl::removeNaNFromPointCloud(*pt_cloud, *pt_cloud, ind);
            // int pointno = 1;
            for (pcl::PointXYZI point : nonground->points) { 
                // std::cout<<"BBBB"<<pointno++<<std::endIter_Cluster.Avg.xl;
                if (isfinite(point.x) && isfinite(point.y) && isfinite(point.z)){
                    int found_cluster = 0;
                    for (int index = 0; index < Clusters_Vector.size(); index++){
                        CustomCluster Iter_Cluster = Clusters_Vector[index];
                        if (
                            check_distance(Clusters_Vector[index].Last, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Left, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Right, point, EuclideanThreshold) ||
                            check_distance(Clusters_Vector[index].Avg, point, CentroidThreshold)
                            ){
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
                            Clusters_Vector[index].Left =  (Iter_Cluster.Left.y >= point.y) ? point: Iter_Cluster.Left ;
                            Clusters_Vector[index].Right = (Iter_Cluster.Right.y <= point.y) ? point: Iter_Cluster.Right;

                            Clusters_Vector[index].Avg.x = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.x + (double)(point.x*inv_current_size);
                            Clusters_Vector[index].Avg.y = (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.y + (double)(point.y*inv_current_size);
                            Clusters_Vector[index].Avg.z = //(Iter_Cluster.Avg.z < point.z) ? Iter_Cluster.Avg.z : point.z;
                            (double)(1 - inv_current_size)*(double)Iter_Cluster.Avg.z + (double)(point.z*inv_current_size);
                            
                            // (Clusters_Vector[index].gradient[(int)(point.z*100)].Value)+= point.intensity;
                            // (Clusters_Vector[index].gradient[(int)(point.z*100)].size)++;
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
                            if (Iter_Cluster.minheight.z > point.z){
                                Clusters_Vector[index].minheight = point;
                            }
                            if (Iter_Cluster.maxheight.z < point.z){
                                Clusters_Vector[index].maxheight = point;
                            }
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
            // cluster_pc->push_back(pcl::PointXYZI(0.f));
            float denominator; float term1; float term2; float term3;
            for (int index = 0; index < Clusters_Vector.size(); index++){
                CustomCluster Iter_Cluster = Clusters_Vector[index];
                if (Iter_Cluster.Avg.x == 0.0 && Iter_Cluster.Avg.y == 0.0 && Iter_Cluster.Avg.z == 0.0 ) continue;
                int dist_sq = pow(Iter_Cluster.Avg.x, 2.0) + pow(Iter_Cluster.Avg.y, 2.0) + pow(Iter_Cluster.Avg.z, 2.0);
                int expected_points = (5000)/(dist_sq + 1); //remove dist =0
                //(height * width)/(8 * distance^2 * tan(vert.reso/2) * tan(hori.reso/2))

                //std::cout <<std::endl<< maxheight<<minheight<<std::endl;
                // std::cout <<pow(dist_sq,0.5)<<" "<< expected_points <<" "<< Iter_Cluster.size<<std::endl;
                // int curr_colour = 1;
                // int dip = 0;
                // for (auto it = Iter_Cluster.gradient.begin()++; it!=Iter_Cluster.gradient.end(); ++it){
                //     auto prev_it = it;
                //     // std::cout<<"1";
                //     --prev_it;
                //     std::cout<<it->first<<" "<<it->second.Value/it->second.size<< " " << it->second.size<<std::endl;
                //     if((it->second.Value)/(it->second.size) > 2.5*(prev_it->second.Value)/(prev_it->second.size)){
                //         if (dip == 1){
                //             curr_colour =0;
                //             break;
                //         }
                //         // else{
                //         //     curr_colour = 0;
                //         // }
                //     }
                //     if(2.5*(it->second.Value)/(it->second.size) < (prev_it->second.Value)/(prev_it->second.size)){
                //         // curr_colour =0;
                //         dip = 1;
                //     }
                // }
                // cone check (add gradient) 


                //checking cone distance from amz lines
                // std::cout<<"Cone"<<std::endl;
                float angleclus = atan2(Iter_Cluster.Avg.x, -Iter_Cluster.Avg.y) - M_PI/6;
                int key = int((angleclus)/(sectorangle));
                if(line.find(key)==line.end()){
                  std::cout << angleclus << std::endl;
                  std::cout << sectorangle << std::endl;
                  std::cout << key << std::endl;
                  std::cout << "bruh "<<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z<<" "<<std::endl;
                  cluster_pc->push_back(Iter_Cluster.Avg);
                  perception::Coordinates CurrentCluster;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                  Cluster.ConeCoordinates.push_back(CurrentCluster);

                }
                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.Avg.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.Avg.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.Avg.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.Avg.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.Avg.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.Avg.x-lidarpoint.x), 2);
                float distance = (pow(term1 + term2 + term3, 0.5))/(denominator);

                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.minheight.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.minheight.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.minheight.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.minheight.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.minheight.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.minheight.x-lidarpoint.x), 2);
                float minzfromground = (pow(term1 + term2 + term3, 0.5))/(denominator);

                denominator = pow(pow(lidarpoint.x-line[key][0], 2) + pow(lidarpoint.y-line[key][1], 2) + pow(lidarpoint.z-line[key][2], 2), 0.5);
                term1 = pow((line[key][1] - lidarpoint.y)*(Iter_Cluster.maxheight.z - lidarpoint.z) - (line[key][2]-lidarpoint.z)*(Iter_Cluster.maxheight.y-lidarpoint.y), 2);
                term2 = pow((line[key][2] - lidarpoint.z)*(Iter_Cluster.maxheight.x - lidarpoint.x) - (line[key][0]-lidarpoint.x)*(Iter_Cluster.maxheight.z-lidarpoint.z), 2);
                term3 = pow((line[key][0] - lidarpoint.x)*(Iter_Cluster.maxheight.y - lidarpoint.y) - (line[key][1]-lidarpoint.y)*(Iter_Cluster.maxheight.x-lidarpoint.x), 2);
                float maxzfromground = (pow(term1 + term2 + term3, 0.5))/(denominator);

                if ( //checks
                // 1
                (distance >=0.1 && distance < 0.22)
                // && (Iter_Cluster.Avg.z + LidarHeight < MaxHeight)
                // && (Iter_Cluster.Avg.z + LidarHeight > MinHeight)
                && (maxzfromground >= 0.26 )
                && (minzfromground <= 0.17 )
                // && (Iter_Cluster.clustersize < expected_points)
                // && (Iter_Cluster.clustersize > 0.13 * expected_points)
                // && ((Iter_Cluster.Right.y - Iter_Cluster.Left.y)*(Iter_Cluster.Right.y - Iter_Cluster.Left.y) + (Iter_Cluster.Right.x - Iter_Cluster.Left.x)*(Iter_Cluster.Right.x - Iter_Cluster.Left.x) < MaxWidth*MaxWidth)
                // && (Iter_Cluster.clustersize > MinPoints)
                // && (Iter_Cluster.Avg.x*Iter_Cluster.Avg.x + Iter_Cluster.Avg.y*Iter_Cluster.Avg.y < 30)
                // && (curr_colour == 0)
                ){
                    Cluster.size++;
                    perception::Coordinates CurrentCluster;
                    CurrentCluster.x = Iter_Cluster.Avg.x;
                    CurrentCluster.y = Iter_Cluster.Avg.y;
                    CurrentCluster.z = Iter_Cluster.Avg.z;
                    // CurrentCluster.colour = curr_colour;
                    std::cout << "Cone " <<Iter_Cluster.clustersize<<" "//<<curr_colour<<" "
                    <<Iter_Cluster.Avg.x<<" "<<Iter_Cluster.Avg.y<<" "<<Iter_Cluster.Avg.z
                    <<" "<<distance <<" "<<minzfromground<<" "<<maxzfromground
                    <<" "<<std::endl; 
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
    // ros::NodeHandle ClusteringHandle;        // ROS node handle
    ros::Publisher Clusters;        // ROS publisher for JointState messages
    ros::Publisher Clusters_pc;
    // ros::Publisher pub;
    ros::Subscriber sub;
    // ros::Publisher All_Clusters_pc;
    ros::Subscriber Subscribe;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Amz");
    ros::NodeHandle ClusteringHandle;
    pub = ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>> ("ConeCloud", 1);
    // pub_ = ClusteringHandle.advertise<perception::CoordinateList>("Clusters", 1);
    // pub_cloud= ClusteringHandle.advertise<pcl::PointCloud<pcl::PointXYZI>>("ClustersCloud", 1);
    marker_pub = ClusteringHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
    Clustering node(ClusteringHandle);
    ros::spin();
    return 0;
}
