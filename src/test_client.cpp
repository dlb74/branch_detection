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
#include <pcl/filters/passthrough.h>

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

#define BRANCH_NORM_KSEARCH_RADIUS 0.01
#define BRANCHSEG_NORMDIST_WEIGHT 0.05
#define BRANCHSEG_MAXITT 1000
#define BRANCHSEG_CYLDIST_THRESH 0.01
#define BRANCHSEG_CYLRAD_MIN 0.002
#define BRANCHSEG_CYLRAD_MAX 0.025

typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;


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

void Branch_Seg( pcl::PointCloud<PointT>::Ptr cloud,
                 pcl::PointCloud<PointNT>::Ptr cloud_normals,
                 pcl::ModelCoefficients::Ptr coefficients_cylinder,
                 pcl::PointCloud<PointT>::Ptr cloud_remainder )
{
    pcl::PointIndices::Ptr inliers_branch (new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
    pcl::ExtractIndices<PointT> extract;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (BRANCHSEG_NORMDIST_WEIGHT);
    seg.setMaxIterations (BRANCHSEG_MAXITT);
    seg.setDistanceThreshold (BRANCHSEG_CYLDIST_THRESH);
    seg.setRadiusLimits (BRANCHSEG_CYLRAD_MIN, BRANCHSEG_CYLRAD_MAX);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_branch, *coefficients_cylinder);

    if (coefficients_cylinder->values[4] < 0) {
        Eigen::Matrix3f R;
        Eigen::Vector3f axis(coefficients_cylinder->values[3],
                             coefficients_cylinder->values[4],
                             coefficients_cylinder->values[5]);
        R << -1, 0, 0,
                0, -1, 0,
                0, 0, -1;
        axis = R*axis;
        coefficients_cylinder->values[3] = axis(0);
        coefficients_cylinder->values[4] = axis(1);
        coefficients_cylinder->values[5] = axis(2);
    }

    // Extract cylinder inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_branch);
    extract.setNegative (true);
    extract.filter (*cloud_remainder);
}

void Filter_Far_Points(pcl::PointCloud<PointT>::Ptr cloud,
                       pcl::PointCloud<PointT>::Ptr cloud_filtered ) {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);

}

/** PCL Frame Normal Estimation */
void Norm_Est( pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_normals, float normKSearchRadius )
{
    pcl::NormalEstimation<PointT, PointNT> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setRadiusSearch ( normKSearchRadius );
    ne.compute (*cloud_normals);
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
    pcl::ModelCoefficients::Ptr coefficients_cylinder_branch (new pcl::ModelCoefficients);
    coefficients_cylinder_branch->values.resize (7);
    pcl::PointCloud<PointNT>::Ptr branch_normals (new pcl::PointCloud<PointNT>);
    pcl::PointCloud<PointT>::Ptr cloud_remainder (new pcl::PointCloud<PointT> ());


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer->initCameraParameters( );
    viewer->setShowFPS( false );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud, "Filtered Cloud");

    while (!viewer->wasStopped ())
    {
        cloud_DownSampled->clear();
        cloud_filtered->clear();
        branch_normals->clear();
        cloud_remainder->clear();

        DownSample( cloud, cloud_DownSampled );
        cloud_DownSampled->width = (int)cloud_DownSampled->points.size();

        Filter_Far_Points( cloud_DownSampled, cloud_filtered );
        //Frame_Filter( cloud, cloud_filtered );
        cloud_filtered->width = (int)cloud_filtered->points.size();

        Norm_Est( cloud_filtered, branch_normals, BRANCH_NORM_KSEARCH_RADIUS );
        branch_normals->width = (int)branch_normals->points.size();

        Branch_Seg( cloud_filtered, branch_normals,
                    coefficients_cylinder_branch, cloud_remainder);

        viewer->removeAllShapes();
        viewer->updatePointCloud( cloud_filtered, "Filtered Cloud" );
        viewer->addCylinder ( *coefficients_cylinder_branch, "sadfsaf" );
        viewer->spinOnce (100);

        cloud->clear();


        ros::spinOnce();

    }
}