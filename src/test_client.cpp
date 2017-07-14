#include "ros/ros.h"
#include "branch_detection/AddTwoInts.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>

#define DWNSMPL_SQRLEAF_SIZE 0.04

#define KMEANFILTER_RANGE 10
#define KMEANFILTER_THRESH_STDVMUL 0.8

typedef pcl::PointXYZ PointT;


pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

/** Downsampling the point cloud */
void DownSample( pcl::PointCloud<PointT>::Ptr cloud,
                 pcl::PointCloud<PointT>::Ptr cloud_DownSampled )
{
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize ( DWNSMPL_SQRLEAF_SIZE, DWNSMPL_SQRLEAF_SIZE, DWNSMPL_SQRLEAF_SIZE );
    sor.filter (*cloud_DownSampled);
}


/** PCL Frame Filtering */
void Frame_Filter( pcl::PointCloud<PointT>::Ptr cloud,
                   pcl::PointCloud<PointT>::Ptr cloud_filtered )
{
    // K-mean Statistical filtering
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (KMEANFILTER_RANGE);
    sor.setStddevMulThresh (KMEANFILTER_THRESH_STDVMUL);
    sor.filter (*cloud_filtered);
}

void chatterCallback(sensor_msgs::PointCloud2ConstPtr msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    ROS_INFO("here");
    pcl::fromROSMsg (*msg, *cloud);
    //cloud_filtered = msg;


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<branch_detection::AddTwoInts>("add_two_ints");
    branch_detection::AddTwoInts srv;

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1000, chatterCallback);

    pcl::PointCloud<PointT>::Ptr cloud_DownSampled (new pcl::PointCloud<PointT>);
    Eigen::Vector3f COM;
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer->initCameraParameters( );
    viewer->setShowFPS( false );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud, "Filtered Cloud");

    while (!viewer->wasStopped ())
    {
        cloud_DownSampled->clear();
        cloud_filtered->clear();

        DownSample( cloud, cloud_DownSampled );
        cloud_DownSampled->width = (int)cloud_DownSampled->points.size();

        Frame_Filter( cloud_DownSampled, cloud_filtered );
        cloud_filtered->width = (int)cloud_filtered->points.size();

        viewer->removeAllShapes();
        viewer->updatePointCloud( cloud_filtered, "Filtered Cloud" );
        viewer->spinOnce (100);

        cloud->clear();


        ros::spinOnce();

    }
}