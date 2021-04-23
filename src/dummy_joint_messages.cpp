//
// Created by werner on 19-04-21.
//

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <geometry_msgs/PointStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dummy_joint_messages", 0);
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher joint_states = nh.advertise<sensor_msgs::JointState>("/joint_states", 1000);


    sensor_msgs::JointState js;
    js.header.seq = 0;
    js.header.frame_id = "";
//    js.name.push_back("dummy_joint");
//    js.position.push_back(0.0);

    while (ros::ok()) {

        ++js.header.seq;
        js.header.stamp = ros::Time::now();
        joint_states.publish(js);

        ros::Duration(0.5).sleep();
    }

    return 0;
}