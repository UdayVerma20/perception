#include "ros/ros.h"
#include <string>
#include <sensor_msgs/PointCloud2.h>
//don 
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/don.h>
//convert
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
//publish 
#include <pcl_ros/point_cloud.h>

using namespace pcl;

//typedef pcl::PointCloud<pcl::PointXYZ> pcl::PointCloud<pcl::PointXYZ>

class DonSegmentation
{
public:
    DonSegmentation()
    {
        //DonSegmentationHandle{};
        //Ground = DonSegmentationHandle.advertise<pcl::PointCloud<pcl::PointNormal>>("Ground", 10);
        PointCloud = DonSegmentationHandle.advertise<pcl::PointCloud<pcl::PointNormal>>("ConeCloud", 10);
        Subscribe = DonSegmentationHandle.subscribe("VoxelCloud", 10, &DonSegmentation::callback, this);
        //Subscribe = DonSegmentationHandle.subscribe("/velodyne_points", 10, &DonSegmentation::callback, this);
        //timer(DonSegmentationHandle.createTimer(ros::Duration(0.1), &DonSegmentation::main_loop, this));
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg ) 
    {
        ///The smallest scale to use in the DoN filter.
        double scale1 = 0.1;

        ///The largest scale to use in the DoN filter.
        double scale2 = 2;

        ///The minimum DoN magnitude to threshold by
        double threshold = 0.3; 

        ///segment scene into clusters with given distance tolerance using euclidean clustering
        //double segradius = 4;

        // Load cloud in blob format
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*msg,pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

        // Create a search tree, use KDTreee for non-organized data.
        pcl::search::Search<PointXYZ>::Ptr tree;
        if (cloud->isOrganized ())
        {
          tree.reset (new pcl::search::OrganizedNeighbor<PointXYZ> ());
        }
        else
        {
          tree.reset (new pcl::search::KdTree<PointXYZ> (false));
        }

        // Set the input pointcloud for the search tree
        tree->setInputCloud (cloud);

        if (scale1 >= scale2)
        {
          std::cerr << "Error: Large scale must be > small scale!" << std::endl;
          exit (EXIT_FAILURE);
        }

        // Compute normals using both small and large scales at each point
        pcl::NormalEstimationOMP<PointXYZ, PointNormal> ne;
        ne.setInputCloud (cloud);
        ne.setSearchMethod (tree);

        /**
         * NOTE: setting viewpoint is very important, so that we can ensure
         * normals are all pointed in the same direction!
         */
        ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

        // calculate normals with the small scale
        //std::cout << "Calculating normals for scale..." << scale1 << std::endl;
        pcl::PointCloud<PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<PointNormal>);

        ne.setRadiusSearch (scale1);
        ne.compute (*normals_small_scale);

        // calculate normals with the large scale
        //std::cout << "Calculating normals for scale..." << scale2 << std::endl;
        pcl::PointCloud<PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<PointNormal>);

        ne.setRadiusSearch (scale2);
        ne.compute (*normals_large_scale);

        // Create output cloud for DoN results
        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<PointNormal>);
        copyPointCloud (*cloud, *doncloud);

        //std::cout << "Calculating DoN... " << std::endl;
        // Create DoN operator
        pcl::DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
        don.setInputCloud (cloud);
        don.setNormalScaleLarge (normals_large_scale);
        don.setNormalScaleSmall (normals_small_scale);

        if (!don.initCompute ())
        {
          std::cerr << "Error: Could not initialize DoN feature operator" << std::endl;
          exit (EXIT_FAILURE);
        }

        // Compute DoN
        don.computeFeature (*doncloud);

        // Build the condition for filtering
        pcl::ConditionOr<PointNormal>::Ptr range_cond (
          new pcl::ConditionOr<PointNormal> ()
          );
        range_cond->addComparison (pcl::FieldComparison<PointNormal>::ConstPtr (
                                    new pcl::FieldComparison<PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
                                  );
        // Build the filter
        pcl::ConditionalRemoval<PointNormal> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (doncloud);

        pcl::PointCloud<PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<PointNormal>);

        // Apply filter
        condrem.filter (*doncloud_filtered);

        doncloud = doncloud_filtered;

        // Save filtered output
        std::cout << "Filtered Pointcloud: " << doncloud->size () << std::endl;
        PointCloud.publish(*doncloud);
        
      }
    

private:
    ros::NodeHandle DonSegmentationHandle;        // ROS node handle
    ros::Publisher PointCloud;
    //ros::Publisher Ground;        // ROS publisher for JointState messages
    ros::Subscriber Subscribe;       // ROS subscriber for a topic
    //ros::Timer timer;          // ROS timer for periodic tasks
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DonSegmentation");
    DonSegmentation node;
    ros::spin();
    return 0;
}
