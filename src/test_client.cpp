#include "ros/ros.h"
#include "branch_detection/AddTwoInts.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>

void chatterCallback(sensor_msgs::PointCloud2ConstPtr msg)
{
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
    ROS_INFO("here");

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_client");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<branch_detection::AddTwoInts>("add_two_ints");
    branch_detection::AddTwoInts srv;

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("camera/depth/points", 1000, chatterCallback);
    
    ros::spin();

    return 0;
}