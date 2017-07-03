#include "ros/ros.h"
#include "branch_detection/AddTwoInts.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool add(branch_detection::AddTwoInts::Request  &req, branch_detection::AddTwoInts::Response &res)  {

    res.sum = req.a + req.b;
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true;
}

int main(int argc, char **argv) {
   ros::init(argc, argv, "branch_detection_server");
   ros::NodeHandle n;

    //Create a ROS subscriber for the input point cloud
   //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
   ROS_INFO("Ready to add two ints.");
   ros::spin();

   return 0;
}