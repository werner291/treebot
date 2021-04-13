#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>

static

void chatterCallback(const geometry_msgs::PointStamped& msg)
{


    ROS_INFO("i : [%f,%f,%f]", msg.point.x, msg.point.y, msg.point.z);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "treebot_controller", 0);
    ros::NodeHandle nh;

    ros::Subscriber rgb = nh.subscribe("/target_xyz", 1000, chatterCallback);

    ros::spin();

    return 0;
}